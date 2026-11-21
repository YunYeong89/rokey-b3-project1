"""Accelerated MuJoCo session without the Doosan emulator (run_virtual.sh --acc [FACTOR]): the emulated controller
(controller.py) makes the nominal joints in simulation time and the unchanged force control (virtual_force) runs on
the simulation clock, so physics, motions and force / stiffness ramps all run FACTOR times faster than real time
(0: as fast as the CPU allows; slower when it cannot keep up). The robot programs still wait in wall time: their
polls see a motion end sooner, their fixed sleeps last FACTOR times longer in simulation time."""
import threading
import time

from virtual_force import GazeboForceBackend
from .backend import MuJoCoForceBackend
from .controller import EmulatedController, Rejected


class TrajectoryTargets:
    """Physics joint targets: the controller's trajectory with its velocity / acceleration while following, one
    control period of interpolation under force control or the surface stream (as backend.JointTargets)."""

    def __init__(self, backend):
        self.backend = backend

    def set(self, positions):
        b = self.backend
        if b.surface.active or b.mode == 'compliance':
            b.engine.reference.set_servo(positions, b.period)
        elif b.mode == 'following':
            b.engine.reference.set(positions, b.nominal_velocity, b.nominal_acceleration)
        else:
            b.engine.reference.set(positions)

    def close(self):
        pass


class AcceleratedBackend(MuJoCoForceBackend):
    REPORT_S = 10.  # wall seconds between speed reports

    def __init__(self, model, profiles, factor):
        self.factor, self.prepared = factor, threading.Event()
        super().__init__(model)  # starts run(), which waits for the controller
        engine = self.engine
        initial = dict(zip(engine.joints, engine.data.qpos[engine.qpos_indices]))
        self.controller = EmulatedController(model.robot, profiles, initial)
        self.ramp = TrajectoryTargets(self)
        self.nominal_stable_since = self.clock()
        self.reported = time.monotonic()
        self.prepared.set()

    def clock(self):
        return self.engine.data.time

    def run(self):
        self.prepared.wait()
        steps = round(self.period / self.engine.model.opt.timestep)
        wall, simulated = time.monotonic(), self.engine.data.time
        try:
            while not self.closed.is_set():
                with self.lock:
                    self.tick(steps)
                    self.measure()
                if self.factor > 0:
                    lead = wall + (self.engine.data.time - simulated) / self.factor - time.monotonic()
                    if lead > 0:
                        self.closed.wait(lead)
                    elif lead < -.1:  # behind: carry on from now rather than catch up in a burst
                        wall, simulated = time.monotonic(), self.engine.data.time
                else:
                    time.sleep(0)  # let ROS commands and snapshots take the lock
        except Exception as error:
            self.error = f'MuJoCo controller stopped: {error}'
            self.closed.set()

    def tick(self, steps):
        """One control period: controller -> nominal, force control -> physics targets, physics steps."""
        positions, self.nominal_velocity, self.nominal_acceleration = self.controller.advance(self.period)
        GazeboForceBackend.update(self, positions)
        self.nominal_available = True
        self.step(self.period)
        for _ in range(steps):
            self.feedback, self.feedback_velocity, raw = self.engine.step()
            self.raw_wrench = raw
            self.raw_sum += raw
            self.raw_count += 1
        self.joint_time = self.sensor_time = self.clock()

    def measure(self):
        now = time.monotonic()
        if now - self.rate_start >= 1.:
            self.realtime_factor = (self.engine.data.time - self.rate_simulation_start) / (now - self.rate_start)
            self.rate_start, self.rate_simulation_start = now, self.engine.data.time
        if now - self.reported >= self.REPORT_S:
            self.reported = now
            target = f'{self.factor:g}x' if self.factor > 0 else 'max'
            print(f'MuJoCo {self.realtime_factor:.1f}x real time (target {target})', flush=True)

    def update(self, positions):
        """The session loop's ROS positions: none without the emulator; only the viewer follows the physics."""
        if self.closed.is_set():
            raise RuntimeError(self.error)
        self.engine.sync_viewer(self.lock)

    def command(self, name, request):
        if name in self.controller.handlers:
            with self.lock:
                try:
                    return {'success': True, 'values': self.controller.command(name, request, self.feedback)}
                except Rejected as error:
                    return {'success': False, 'message': str(error)}
        if name == 'resume':
            with self.lock:
                if self.hold:
                    self.controller.reset(self.hold)  # the nominal takes over where the robot is held
        return super().command(name, request)

    def snapshot(self):
        with self.lock:
            return {**super().snapshot(), 'controller': self.controller.state(), 'acc_factor': self.factor}
