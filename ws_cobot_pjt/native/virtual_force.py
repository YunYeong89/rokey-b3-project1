"""Virtual-only Cartesian admittance and force feedback over native Gazebo."""
import socket
import threading
import subprocess
import time
from pathlib import Path

import numpy as np
import yaml

from displays import GazeboDisplay, StateReceiver
from kinematics import KinematicChain, rotation, rotation_error
from setpoint_ramp import SetpointRamp
from surface_stream import CartesianStream, StaleSetpoint


class GazeboForceBackend(GazeboDisplay):
    # Control time of the freshness checks and force / stiffness transitions: wall time here, simulation time in
    # the accelerated MuJoCo session (dsr_mujoco/accelerated.py).
    clock = staticmethod(time.monotonic)

    def __init__(self, model):
        self.lock = threading.RLock()
        super().__init__(model)
        from gz.msgs10.wrench_pb2 import Wrench
        from gz.transport13 import SubscribeOptions
        from gz.transport13 import AdvertiseMessageOptions
        self.Wrench = Wrench
        self.chain = KinematicChain(model.robot, tcp_link=model.tcp_link)
        self.sensor_chain = KinematicChain(model.robot, tcp_link='tool0')
        self.probe = None
        if model.scene_map is not None and model.scene_map.probe_tool is not None:
            from probe_grasp import ProbeGrasp
            self.probe = ProbeGrasp(self, model.scene_map.probe_tool)
        self.gravity_publishers = {name: self.node.advertise(
            f'/model/cobot/joint/{name}/cmd_force', 'gz.msgs.Double', AdvertiseMessageOptions())
            for name in self.chain.names}
        settings = yaml.safe_load(Path(__file__).with_suffix('.yaml').read_text())
        self.period = 1 / settings['control_hz']
        self.mass = np.asarray(settings['mass'], dtype=float)
        self.minimum_damping = np.asarray(settings['minimum_damping'], dtype=float)
        self.maximum_velocity = np.asarray(settings['maximum_velocity'], dtype=float)
        self.maximum_displacement = np.asarray(settings['maximum_displacement'], dtype=float)
        self.maximum_joint_velocity = settings['maximum_joint_velocity']
        self.wrench_filter_time = settings['wrench_filter_time']
        self.nominal = {}
        self.nominal_available = False
        self.nominal_time = self.sensor_time = 0.0
        self.nominal_stable_since = self.clock()
        self.raw_wrench = np.zeros(6)
        self.raw_sum, self.raw_count = np.zeros(6), 0
        self.compensated_wrench = np.zeros(6)
        self.joint_time = 0.0
        self.wrench = np.zeros(6)
        self.bias = None
        self.calibration = []
        self.mode = 'following'
        self.reference = np.eye(3)
        self.stiffness = np.array([3000., 3000., 3000., 200., 200., 200.])
        self.stiffness_start = self.stiffness.copy()
        self.stiffness_target = self.stiffness.copy()
        self.transition_start = self.transition_duration = 0.0
        self.force_start = self.force_target = np.zeros(6)
        self.release_pending = False
        self.force_transition_start = self.force_transition_duration = 0.0
        self.direction = np.zeros(6, dtype=bool)
        self.offset = np.zeros(6)
        self.velocity = np.zeros(6)
        self.bias_position = np.zeros(3)
        self.bias_rotation = np.eye(3)
        self.hold = None
        self.arm_command = {}
        self.force_nominal_pose = None
        self.sync_target = None
        self.error = ''
        self.surface = CartesianStream(self)
        self.surface_sink = None  # set by ForceStateReceiver: each control tick's surface state goes out at once
        self.ramp = SetpointRamp(super().update, self.period)
        self.closed = threading.Event()
        self.node.subscribe_raw('/cobot/force_torque', self.receive_wrench,
                                'gz.msgs.Wrench', SubscribeOptions())
        self.worker = threading.Thread(target=self.run, daemon=True)
        self.worker.start()

    def receive(self, data, info):
        with self.lock:
            super().receive(data, info)
            self.joint_time = self.clock()

    def receive_wrench(self, data, _info):
        message = self.Wrench.FromString(data)
        with self.lock:
            raw = np.array([message.force.x, message.force.y, message.force.z,
                            message.torque.x, message.torque.y, message.torque.z])
            if not np.isfinite(raw).all():
                self.sensor_time = 0
                self.error = 'Invalid force sensor sample'
                return
            self.raw_wrench = raw
            self.raw_sum += raw
            self.raw_count += 1
            self.sensor_time = self.clock()

    def update(self, positions):
        with self.lock:
            if any(abs(positions.get(name, 0) - self.nominal.get(name, 0)) > 1e-5
                   for name in self.chain.names):
                self.nominal_stable_since = self.clock()
            self.nominal = positions.copy()
            self.nominal_time = self.clock()

    def ready(self):
        return (all(name in self.feedback and name in self.nominal for name in self.chain.names)
                and self.nominal_available and not self.closed.is_set()
                and self.clock() - self.nominal_time < 0.5
                and self.clock() - self.sensor_time < 0.25 and self.bias is not None)

    def frame(self, reference):
        if reference in (0, 2):
            return np.eye(3)
        if reference == 1:
            return self.chain.pose_jacobian(self.feedback)[0][:3, :3]
        raise ValueError('Virtual force control supports BASE, WORLD and TOOL references')

    def anchor(self):
        actual = self.chain.pose_jacobian(self.feedback)[0]
        nominal = self.chain.pose_jacobian(self.nominal)[0]
        self.bias_position = actual[:3, 3] - nominal[:3, 3]
        self.bias_rotation = actual[:3, :3] @ nominal[:3, :3].T
        self.offset[:] = 0
        self.velocity[:] = 0
        self.arm_command = {name: self.feedback[name] for name in self.chain.names}

    def command(self, name, request):
        if name == 'attach_probe':
            # Gazebo transport callbacks must remain free while the handoff
            # pauses physics and waits for joint-state acknowledgements.
            with self.lock:
                ready, positions = self.ready(), self.feedback.copy()
            if not ready or self.probe is None:
                return {'success': False, 'message': 'Rack probe / Gazebo feedback is not ready'}
            try:
                result = self.probe.attach(positions)
            except (TimeoutError, subprocess.SubprocessError) as error:
                return {'success': False, 'message': str(error)}
            if result['success']:
                with self.lock:
                    # The held probe's ball bottom (probe_tip = ProbeTCP) is the TCP from now on: the compliance
                    # centre and tcp_offset_mm (added to the emulator's ProbeTCP posx) were at the gripper TCP,
                    # 52.7 mm short of it, so a tool tilt of d rad showed as d x 52.7 mm of position error.
                    self.chain = KinematicChain(self.model.robot, tcp_link='probe_tip')
                    self.surface.chain = KinematicChain(self.model.robot, tcp_link='probe_tip')
            return result
        with self.lock:
            if name in ('cartesian_start', 'cartesian_stop'):
                try:
                    if name == 'cartesian_start':
                        return self.surface.begin('position', request.get('pd'), force_axis=request.get('force_axis'))
                    if self.surface.method != 'position' or request['run_id'] != self.surface.run_id:
                        raise ValueError('Cartesian stream ownership mismatch')
                    return self.surface.halt('Cartesian stream released by operator')
                except (ValueError, KeyError) as error:
                    return {'success': False, 'message': str(error)}
            if not self.ready():
                return {'success': False, 'message': 'Gazebo joint / force feedback is not ready'}
            try:
                now = self.clock()
                if name == 'surface_hybrid':
                    return self.surface.begin('hybrid')
                if name == 'surface_end':
                    return self.surface.halt('Surface stream released by operator')
                if self.surface.active:
                    raise ValueError('Release the surface stream before using legacy force services')
                if name == 'task_compliance_ctrl':
                    reference = self.frame(request['ref'])
                    self.set_stiffness(request, now)
                    self.reference = reference
                    self.anchor()
                    self.direction[:] = False
                    self.force_nominal_pose = None
                    self.release_pending = False
                    self.force_start = self.force_target = np.zeros(6)
                    self.mode, self.error = 'compliance', ''
                elif name == 'set_stiffnessx':
                    if self.mode != 'compliance':
                        raise ValueError('Enable task_compliance_ctrl first')
                    reference = self.frame(request['ref'])
                    self.set_stiffness(request, now)
                    self.relatch(reference, self.force_nominal_pose, self.direction)
                elif name == 'set_desired_force':
                    if self.mode != 'compliance':
                        raise ValueError('Enable task_compliance_ctrl first')
                    reference = self.frame(request['ref'])
                    direction = np.asarray(request['dir'])
                    force = np.asarray(request['fd'], dtype=float)
                    if direction.shape != (6,) or not np.isin(direction, [0, 1]).all() or not direction.any():
                        raise ValueError('dir requires six binary values and at least one force axis')
                    if force.shape != (6,) or not np.isfinite(force).all():
                        raise ValueError('fd requires six finite values')
                    if request['mod'] not in (0, 1):
                        raise ValueError('Unsupported force mode')
                    duration = self.duration(request)
                    measured = self.rotate_wrench(reference.T, self.wrench)
                    self.force_start = self.target_force(now)
                    self.force_start[~self.direction] = -measured[~self.direction]
                    self.relatch(reference, self.chain.pose_jacobian(self.nominal)[0], direction.astype(bool))
                    self.force_target = force - measured if request['mod'] == 1 else force
                    self.force_transition_start = now
                    self.force_transition_duration = duration
                    self.release_pending = False
                elif name == 'release_force':
                    if self.mode != 'compliance':
                        raise ValueError('Compliance control is not active')
                    duration = self.duration(request)
                    self.force_start = self.target_force(now)
                    self.force_target = np.zeros(6)
                    self.force_transition_start, self.force_transition_duration = now, duration
                    self.release_pending = True
                elif name == 'release_compliance_ctrl':
                    self.hold = {key: self.feedback[key] for key in self.chain.names}
                    self.mode = 'holding'
                    self.release_pending = False
                    return {'success': True, 'sync_q': [self.hold[key] for key in self.chain.names]}
                elif name == 'resume':
                    self.sync_target = self.hold.copy()
                else:
                    raise ValueError(f'Unknown virtual force command: {name}')
                return {'success': True}
            except (ValueError, KeyError) as error:
                return {'success': False, 'message': str(error)}

    @staticmethod
    def duration(request):
        value = float(request.get('time', 0))
        if not np.isfinite(value) or (value != -1 and not 0 <= value <= 1):
            raise ValueError('Transition time must be in [0, 1], or -1')
        return max(0., value)

    def set_stiffness(self, request, now):
        stiffness = np.asarray(request['stx'], dtype=float)
        if stiffness.shape != (6,) or not np.isfinite(stiffness).all() or (stiffness < 0).any():
            raise ValueError('stx requires six finite, nonnegative values')
        if (stiffness > [20000, 20000, 20000, 1000, 1000, 1000]).any():
            raise ValueError('Virtual stiffness exceeds the supported range')
        duration = self.duration(request)
        self.stiffness_start = self.stiffness.copy()
        self.stiffness_target = stiffness
        self.transition_start, self.transition_duration = now, duration

    @staticmethod
    def interpolate(start, target, elapsed, duration):
        return start + (target - start) * (min(1., elapsed / duration) if duration else 1.)

    def target_force(self, now):
        return self.interpolate(self.force_start, self.force_target,
                                now - self.force_transition_start, self.force_transition_duration)

    @staticmethod
    def rotate_wrench(matrix, wrench):
        return np.r_[matrix @ wrench[:3], matrix @ wrench[3:]]

    def target_pose(self):
        """Commanded TCP: nominal (frozen along force axes since the last latch) + bias + offset."""
        nominal_pose = self.chain.pose_jacobian(self.nominal)[0]
        if self.force_nominal_pose is not None:
            initial = self.force_nominal_pose
            delta = self.rotate_wrench(self.reference.T, np.r_[
                nominal_pose[:3, 3] - initial[:3, 3],
                rotation_error(nominal_pose[:3, :3], initial[:3, :3])])
            delta[self.direction] = 0
            nominal_pose[:3, 3] = initial[:3, 3] + self.reference @ delta[:3]
            nominal_pose[:3, :3] = rotation(self.reference @ delta[3:]) @ initial[:3, :3]
        return (nominal_pose[:3, 3] + self.bias_position + self.reference @ self.offset[:3],
                rotation(self.reference @ self.offset[3:]) @ self.bias_rotation @ nominal_pose[:3, :3])

    def relatch(self, reference, force_nominal_pose, direction):
        """Switch control frame / force axes without moving the commanded TCP (bumpless)."""
        position, orientation = self.target_pose()
        self.velocity = self.rotate_wrench(reference.T @ self.reference, self.velocity)
        self.reference, self.force_nominal_pose, self.direction = reference, force_nominal_pose, direction
        self.offset[:] = 0
        base_position, base_orientation = self.target_pose()
        self.offset = self.rotate_wrench(reference.T, np.r_[
            position - base_position, rotation_error(orientation, base_orientation)])

    def step(self, dt):
        now = self.clock()
        actual = self.feedback.copy()
        if not all(name in actual for name in self.chain.names):
            return
        if not np.isfinite([actual[name] for name in self.chain.names]).all():
            raise ValueError('Invalid Gazebo joint position')
        pose, jacobian = self.chain.pose_jacobian(actual)
        sensor_pose, sensor_jacobian = self.sensor_chain.pose_jacobian(actual)
        probe_gravity = self.probe.gravity_wrench(sensor_pose, actual) if self.probe else np.zeros(6)
        gravity_torques = self.chain.gravity_torques(actual) - sensor_jacobian.T @ probe_gravity
        for name, torque in zip(self.chain.names, gravity_torques):
            self.gravity_publishers[name].publish_raw(
                self.Double(data=float(torque)).SerializeToString(), 'gz.msgs.Double')
        # Mean of the 500 Hz samples since the last tick: the latest one alone aliased Gazebo's 2 ms contact
        # impulses into a slow force ripple.
        sample = self.raw_sum / self.raw_count if self.raw_count else self.raw_wrench
        self.raw_sum, self.raw_count = np.zeros(6), 0
        raw = self.rotate_wrench(sensor_pose[:3, :3], sample)
        residual = raw - self.sensor_chain.gravity_wrench(actual) - probe_gravity
        if self.bias is None and self.sensor_time:
            self.calibration.append(residual)
            if len(self.calibration) >= 40:
                self.bias = np.mean(self.calibration, axis=0)
        if self.bias is not None:
            measured = residual - self.bias
            measured[3:] -= np.cross(pose[:3, 3] - sensor_pose[:3, 3], measured[:3])
            self.compensated_wrench = measured.copy()
            self.wrench += (1 - np.exp(-dt / self.wrench_filter_time)) * (measured - self.wrench)
        self.surface.observe(actual, pose, jacobian, now)
        if self.surface_sink:
            self.surface_sink(self.surface_message())
        if not self.nominal:
            return
        if self.surface.active:
            self.ramp.set(self.surface.step(actual, pose, jacobian, dt, now))
            return
        if self.mode == 'compliance' and not self.ready():
            self.hold = {key: actual[key] for key in self.chain.names}
            self.mode, self.error = 'holding', 'Force feedback expired; holding position'
        if self.sync_target and all(abs(self.nominal[name] - self.sync_target[name]) < 0.003
                                    for name in self.chain.names):
            self.sync_target, self.hold, self.mode = None, None, 'following'
        command = self.nominal.copy()
        if self.mode == 'holding':
            command.update(self.hold)
        elif self.mode == 'compliance':
            if self.release_pending and now >= self.force_transition_start + self.force_transition_duration:
                self.anchor()
                self.direction[:] = False
                self.force_nominal_pose = None
                self.force_start = self.force_target = np.zeros(6)
                self.release_pending = False
            self.stiffness = self.interpolate(self.stiffness_start, self.stiffness_target,
                                               now - self.transition_start, self.transition_duration)
            measured = self.rotate_wrench(self.reference.T, self.wrench)
            damping = np.maximum(2.0 * np.sqrt(self.mass * self.stiffness), self.minimum_damping)
            effort = measured - self.stiffness * self.offset
            effort[self.direction] = (measured + self.target_force(now))[self.direction]
            self.velocity += (effort - damping * self.velocity) / self.mass * dt
            self.velocity = np.clip(self.velocity, -self.maximum_velocity, self.maximum_velocity)
            self.offset += self.velocity * dt
            limits = self.maximum_displacement
            saturated = np.abs(self.offset) >= limits
            self.offset = np.clip(self.offset, -limits, limits)
            self.velocity[saturated] = 0
            self.error = 'Cartesian displacement limit reached' if saturated.any() else ''
            target_position, target_rotation = self.target_pose()
            command_seed = {**actual, **self.arm_command}
            command_pose, jacobian = self.chain.pose_jacobian(command_seed)
            error = np.r_[target_position - command_pose[:3, 3],
                          rotation_error(target_rotation, command_pose[:3, :3])]
            increment = jacobian.T @ np.linalg.solve(jacobian @ jacobian.T + np.eye(6)*1e-5, error)
            increment = np.clip(increment, -self.maximum_joint_velocity * dt,
                                self.maximum_joint_velocity * dt)
            self.arm_command = {name: command_seed[name] + increment[index]
                                for index, name in enumerate(self.chain.names)}
            for name, value in self.arm_command.items():
                low, high = self.chain.limits[name]
                if not low <= value <= high:
                    self.arm_command[name] = float(np.clip(value, low, high))
                    self.error = f'Joint position limit reached: {name}'
            command.update(self.arm_command)
        self.ramp.set(command)

    def run(self):
        previous = time.monotonic()
        deadline = previous + self.period
        try:
            while not self.closed.wait(max(0, deadline - time.monotonic())):
                now = time.monotonic()
                with self.lock:
                    self.step(min(now - previous, 0.02))
                previous = now
                deadline += self.period
                if deadline < time.monotonic():
                    deadline = time.monotonic() + self.period
        except Exception as error:
            self.error = f'Virtual controller stopped: {error}'
            self.closed.set()

    def snapshot(self):
        with self.lock:
            from rokey_pose import zyz
            pose = self.chain.pose_jacobian(self.feedback)[0] if self.feedback else np.eye(4)
            nominal = self.chain.pose_jacobian(self.nominal)[0] if self.nominal else pose
            base = self.chain.transforms(self.feedback)[0]['base_link']
            tcp_base = np.linalg.inv(base) @ pose
            return {'positions': self.feedback.copy(), 'wrench': self.wrench.tolist(),
                    'tcp_pose_base_mm_deg': [*(tcp_base[:3, 3] * 1000), *zyz(tcp_base[:3, :3])],
                    # The emulator reports the commanded TCP; this is Gazebo's actual deviation.
                    'tcp_offset_mm': ((pose[:3, 3] - nominal[:3, 3]) * 1000).tolist(),
                    'probe_attached': bool(self.probe and self.probe.design.attached),
                    'tool_rotation': pose[:3, :3].tolist(), 'ready': self.ready(),
                    'mode': self.mode, 'error': self.error,
                    'settled': (self.clock() - self.nominal_stable_since > 0.2 and
                                all(abs(self.feedback_velocity.get(name, float('inf'))) < 0.02
                                    for name in self.chain.names)),
                    'sensor_raw': self.raw_wrench.tolist(), 'offset': self.offset.tolist()}

    def surface_message(self):
        """Surface stream state for rokey, 'kind' first (the bridge forwards the line unparsed)."""
        return {'kind': 'surface', **self.surface.snapshot(), 'ready': self.ready(),
                'motion_ready': self.surface.motion_ready(),
                'probe_attached': bool(self.probe and self.probe.design.attached),
                'backend_mode': self.mode, 'backend_error': self.error}

    def surface_setpoint(self, packet):
        with self.lock:
            try:
                self.surface.accept(packet)
            except StaleSetpoint:
                self.surface.dropped += 1
            except (ValueError, KeyError, TypeError) as error:
                if self.surface.active and packet.get('run_id') == self.surface.run_id:
                    self.surface.halt(str(error))

    def close(self):
        self.closed.set()
        self.worker.join(timeout=2)
        self.ramp.close()


class ForceStateReceiver(StateReceiver):
    def __init__(self, listener, token, backend):
        self.backend = backend
        self.send_lock = threading.Lock()
        self.latest, self.posted = None, threading.Condition()
        super().__init__(listener, token)
        backend.surface_sink = self.post
        threading.Thread(target=self.publish_surface, daemon=True).start()

    def post(self, payload):
        """Latest surface state from the 100 Hz control loop, sent by its own thread. It used to ride on the display
        loop's feedback (~30 ms period behind RViz / scene / interpolation work, stalls of 100+ ms)."""
        with self.posted:
            self.latest = payload
            self.posted.notify()

    def publish_surface(self):
        while not self.closed.is_set():
            with self.posted:
                self.posted.wait_for(lambda: self.latest is not None, timeout=.5)
                payload, self.latest = self.latest, None
            if payload is not None:
                try:
                    self.send(payload)
                except OSError:
                    pass

    def send(self, payload):
        import json
        with self.send_lock:
            if self.stream:
                self.stream.sendall((json.dumps(payload) + '\n').encode())

    def receive(self):
        import json
        try:
            self.stream, _ = self.listener.accept()
            # Small 100 Hz state lines: without it Nagle held them for the container's delayed ACKs (arrivals in
            # bursts, gaps up to 140 ms). The container end already sets it.
            self.stream.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.stream.settimeout(30)
            with self.stream.makefile('r') as file:
                if file.readline().strip() != self.token:
                    raise ConnectionError('Invalid virtual session token')
                for line in file:
                    message = json.loads(line)
                    if message['kind'] == 'positions':
                        with self.backend.lock:
                            self.backend.nominal_available = message['fresh']
                            if hasattr(self.backend, 'receive_nominal'):
                                self.backend.receive_nominal(message['positions'], message['fresh'])
                        with self.lock:
                            self.positions.update(message['positions'])
                            self.interpolator.receive(message['positions'], time.monotonic())
                    elif message['kind'] == 'request':
                        response = self.backend.command(message['command'], message['request'])
                        self.send({'kind': 'response', 'id': message['id'], **response})
                    elif message['kind'] == 'surface_setpoint':
                        self.backend.surface_setpoint(message['request'])
        except OSError:
            pass
        finally:
            self.closed.set()
