"""MuJoCo adapter for the existing native force-control protocol (no Gazebo transport)."""
import threading
import time
from pathlib import Path
import numpy as np
import yaml
from kinematics import KinematicChain
from surface_stream import CartesianStream
from virtual_force import GazeboForceBackend
from .engine import Engine, Scalar, TorqueChannel


class JointTargets:
    def __init__(self, backend):
        self.backend = backend

    def set(self, positions):
        b = self.backend
        if b.surface.active or b.mode == 'compliance':
            b.engine.reference.set_servo(positions, b.period)
        elif b.mode == 'following' and b.streamed_nominal:
            age = min(time.monotonic() - b.nominal_time, .025)
            b.engine.reference.set(positions, b.nominal_velocity, b.nominal_acceleration, age)
        else:
            b.engine.reference.set(positions, {}, {})

    def close(self):
        pass


class MuJoCoForceBackend(GazeboForceBackend):
    def __init__(self, model):
        self.lock = threading.RLock()
        self.model = model
        self.engine = Engine(model)
        self.feedback, self.feedback_velocity = {}, {}
        self.Double = Scalar
        self.chain = KinematicChain(model.robot, tcp_link=model.tcp_link)
        self.sensor_chain = KinematicChain(model.robot, tcp_link='tool0')
        self.probe = None
        if model.scene_map is not None and model.scene_map.probe_tool is not None:
            from .probe import MuJoCoProbeGrasp as ProbeGrasp
            self.probe = ProbeGrasp(self, model.scene_map.probe_tool)
        self.gravity_publishers = {name: TorqueChannel(self.engine, name) for name in self.chain.names}
        settings = yaml.safe_load(Path(__import__('virtual_force').__file__).with_suffix('.yaml').read_text())
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
        self.nominal_stable_since = time.monotonic()
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
        self.streamed_nominal = False
        self.nominal_velocity, self.nominal_acceleration = {}, {}
        self.ramp = JointTargets(self)
        self.realtime_factor = 1.
        self.rate_start = time.monotonic()
        self.rate_simulation_start = self.engine.data.time
        self.closed = threading.Event()
        self.worker = threading.Thread(target=self.run, daemon=True)
        self.worker.start()


    def run(self):
        deadline = time.monotonic()
        physics_steps = round(self.period / self.engine.model.opt.timestep)
        try:
            while not self.closed.wait(max(0., deadline - time.monotonic())):
                with self.lock:
                    for _ in range(physics_steps):
                        self.feedback, self.feedback_velocity, raw = self.engine.step()
                        self.raw_wrench = raw
                        self.raw_sum += raw
                        self.raw_count += 1
                    self.joint_time = self.sensor_time = time.monotonic()
                    self.step(self.period)
                    now = time.monotonic()
                    if now - self.rate_start >= 1.:
                        self.realtime_factor = ((self.engine.data.time - self.rate_simulation_start)
                                                / (now - self.rate_start))
                        self.rate_start, self.rate_simulation_start = now, self.engine.data.time
                time.sleep(0)  # Yield after releasing the lock, including catch-up batches.
                deadline += self.period
                if deadline < time.monotonic() - .05:
                    deadline = time.monotonic()
        except Exception as error:
            self.error = f'MuJoCo controller stopped: {error}'
            self.closed.set()

    def update(self, positions):
        if self.closed.is_set():
            raise RuntimeError(self.error)
        if not self.streamed_nominal:
            super().update(positions)
        self.engine.sync_viewer(self.lock)

    def receive_nominal(self, positions, fresh):
        with self.lock:
            now = time.monotonic()
            elapsed = max(now - self.nominal_time, .001)
            weight = 1 - np.exp(-elapsed / .012)
            for name, value in positions.items():
                old_velocity = self.nominal_velocity.get(name, 0.)
                measured = (value - self.nominal.get(name, value)) / elapsed
                velocity = old_velocity + weight * (measured - old_velocity)
                self.nominal_velocity[name] = velocity
                self.nominal_acceleration[name] = (velocity - old_velocity) / elapsed
            self.nominal_available, self.streamed_nominal = fresh, True
            super().update(positions)

    def snapshot(self):
        with self.lock:
            result = super().snapshot()
            error = max((abs(self.nominal.get(name, value) - value)
                         for name, value in self.feedback.items() if name in self.chain.names), default=0.)
            result['settled'] = result['settled'] and error < .0035
            return {**result, 'simulation_time': self.engine.data.time,
                    'realtime_factor': self.realtime_factor, 'nominal_error_rad': error}

    def close(self):
        super().close()
        self.engine.close()

    def surface_message(self):
        return {**super().surface_message(), 'simulator': 'mujoco',
                'simulation_time': self.engine.data.time, 'realtime_factor': self.realtime_factor,
                'collision_geometry': getattr(self.model, 'collision_info', {})}
