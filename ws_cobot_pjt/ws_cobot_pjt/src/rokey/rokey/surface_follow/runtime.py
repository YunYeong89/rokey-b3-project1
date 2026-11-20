"""HOME -> free-space approach / tare -> one continuous path -> retract -> HOME."""
import time

import numpy as np

from ..program import RobotProgram
from .controllers import HybridFollower
from .geometry import align
from .recording import Recording
from .transport import VirtualTransport


class SurfaceRun:
    def __init__(self, settings, path, program=None, transport=None, recording=None):
        """`program`: an existing RobotProgram to run on (kept open); otherwise one is created and closed."""
        self.s, self.path, self.owned = settings, path, program is None
        self.program = program or RobotProgram('surface_follow')
        self.transport = transport or VirtualTransport(self.program)
        self.recording = recording if recording is not None else Recording(self.program.node, settings, path)

    def prepare(self):
        p, t, s = self.program, self.transport, self.s
        if p.mode != 'virtual':
            raise RuntimeError('surface_follow is restricted to the native Gazebo virtual session')
        p.robot.verify_profiles(p.config['probe']['tool_name'], p.config['probe']['tcp_name'])
        t.wait_ready()
        if not t.data.get('probe_attached'):
            raise RuntimeError('Probe is not attached; run ros2 run rokey pick_probe first')
        if t.data.get('tcp_link') != 'probe_tip':
            raise RuntimeError('Native surface stream must report the probe_tip TCP')
        if t.data.get('active') or t.data.get('backend_mode') != 'following':
            raise RuntimeError('Release the existing virtual controller before starting surface_follow')
        p.log('Surface follow: HOME before approach')
        p.home()
        t.wait_ready()
        t.wait_still()
        t.tare()  # Free space at HOME; never zero away an unexpected contact at the approach point.
        self.approach(t.state())
        state = t.wait_still()
        if np.linalg.norm(state.fast_wrench[:3]) > s.touch:
            raise RuntimeError('Unexpected contact at the clearance pose; inspect the identified vehicle pose')
        # Free space confirmed with the HOME tare; tare again in this orientation, whose gravity-compensation
        # residual differs from the vertical HOME one (0.2-0.45 N at 40 deg gave false contacts).
        t.tare()
        t.begin(s.method)
        state = t.state()  # after begin: the native side rejects setpoints from states older than 150 ms
        self.controller = HybridFollower(s, self.path, state)
        t.send(self.controller.update(state, 1 / s.hz), s.radius)

    def approach(self, state):
        """From HOME: up to the travel height, across above the path start, turn there, straight down to the
        clearance pose."""
        p, s = self.program, self.s
        _, _, normal = self.path.sample(0.)
        orientation = align(state.rotation, normal, np.pi)
        centre = self.path.centres[0] + s.clearance * normal
        tcp = centre + s.radius * orientation[:, 2]
        height = max(p.safe_z() / 1000, state.position[2], tcp[2] + .015)
        motion = p.config['motion']
        p.probe.move([*state.position[:2] * 1000, height * 1000], state.rotation,
                     motion['approach_speed'], acc=motion['approach_acc'])
        p.probe.move([*tcp[:2] * 1000, height * 1000], state.rotation,
                     motion['linear_speed'], acc=motion['linear_acc'])
        p.probe.move([*tcp[:2] * 1000, height * 1000], orientation, motion['approach_speed'])
        p.probe.move(tcp * 1000, orientation, motion['approach_speed'])

    def follow(self, duration):
        self.prepare()
        start = previous = time.monotonic()
        self.started_at = start
        due = previous + 1 / self.s.hz
        last_phase = ''
        while True:
            self.transport.spin(max(0., min(.005, due - time.monotonic())))
            now = time.monotonic()
            if now < due:
                continue
            dt, previous = now - previous, now
            due = now + 1 / self.s.hz
            if dt > .1:
                raise TimeoutError('Surface control scheduling gap exceeded 100 ms')
            if now - start > duration:
                raise TimeoutError('Surface run duration exceeded')
            state = self.transport.state()
            self.transport.check_owner()
            command = self.controller.update(state, dt)
            self.transport.send(command, self.s.radius)
            self.recording.record(now - start, self.controller, state, dt)
            if self.controller.phase != last_phase:
                last_phase = self.controller.phase
                self.program.log(f'{last_phase}: Fn={self.controller.force:.2f} N, '
                                 f'path={self.path.progress * 1000:.1f}/{self.path.length * 1000:.1f} mm')
            if self.controller.phase == 'FINISHED':
                break
        self.retract()

    def retract(self):
        """Only on normal completion. Cancellation holds; it does not attempt an automatic escape."""
        t, s, c = self.transport, self.s, self.controller
        state = t.state()
        position = state.position.copy()
        origin = position.copy()
        previous, deadline = time.monotonic(), time.monotonic() + 8
        c.phase = 'RETRACT'
        c.feed = c.goal = 0.
        while True:
            t.spin(1 / s.hz)
            now = time.monotonic()
            dt, previous = now - previous, now
            state = t.state()
            t.check_owner()
            c.force = float(state.wrench[:3] @ c.normal)
            self.recording.record(now - self.started_at, c, state, dt)
            if dt > .1 or now > deadline or np.linalg.norm(state.fast_wrench[:3]) > s.force_limit:
                raise RuntimeError('Retraction timing / force limit exceeded')
            position += .003 * c.normal * dt
            t.send(dict(position=position.tolist(), rotation=state.rotation.tolist(), force=None,
                        angular_speed=s.angular_speed, angular_acceleration=s.angular_acceleration,
                        source_sequence=state.sequence, source_sampled_at=state.sampled_at), s.radius)
            if (state.position - origin) @ c.normal >= s.clearance:
                break

    def complete_motion(self):
        self.program.home()

    def execute(self, duration):
        outcome, reason, confirmed = 'ABORTED', '', False
        try:
            self.follow(duration)
            outcome = 'SUCCEEDED'
        except KeyboardInterrupt:
            outcome, reason = 'CANCELED', 'Operator Ctrl+C'
        except Exception as error:
            reason = f'{type(error).__name__}: {error}'
        finally:
            native_before_stop = self.transport.data.copy()
            try:
                if self.transport.run_id:
                    confirmed = self.transport.end()
                    if not confirmed:
                        outcome, reason = 'ABORTED', reason + ' Stop motion was not confirmed.'
                if outcome == 'SUCCEEDED' and confirmed:
                    self.complete_motion()
            except Exception as error:
                outcome, reason = 'ABORTED', reason + f' Stop / HOME failed: {error}'
            finally:
                try:
                    directory = self.recording.save(outcome, reason, confirmed, native_before_stop)
                    self.program.log(f'{outcome}: {reason or "path completed"}; saved {directory}')
                finally:
                    if self.owned:
                        self.program.close()
        return 0 if outcome == 'SUCCEEDED' else 1
