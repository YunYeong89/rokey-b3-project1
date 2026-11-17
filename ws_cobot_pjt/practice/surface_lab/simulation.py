"""Multi-rate robot surrogate with delayed measurements, command holds and stop acknowledgement."""
from collections import deque
from dataclasses import asdict
from math import degrees

from pymunk import Vec2d

from .control import AdmittanceFollower, HybridFollower
from .geometry import Terrain, angle_error, normal_angle
from .io_model import DelayLine, RobotDriver, SensorModel, State
from .physics import Command, ProbePhysics, Sensor


class Experiment:
    def __init__(self, settings):
        self.settings = settings
        self.reset()

    def reset(self):
        self.terrain = Terrain(self.settings.scene)
        self.physics = ProbePhysics(self.settings, self.terrain)
        cls = HybridFollower if self.settings.controller == 'hybrid' else AdmittanceFollower
        self.controller = cls(self.settings, self.physics.body.position)
        self.sensor_model = SensorModel(self.settings)
        self.feedback = DelayLine(19)
        self.driver = RobotDriver(self.settings, self.physics.body.position)
        self.time = 0.0
        self.steps = 0
        self.latest = self.produced = State(0, Vec2d(*self.physics.body.position), Vec2d(0, 0), 0, Sensor(), False)
        self.command = self.driver.command
        self.rows, self.points, self.true_points = [], [], []
        self.initial_settings = asdict(self.settings)
        self.last_settings = self.initial_settings.copy()
        self.parameter_changes, self.events = [], []
        self.history = deque(maxlen=1200)
        self.start_x = self.physics.body.position.x
        s = self.settings
        self.periods = {'sensor': 1 / s.sensor_hz, 'feedback': 1 / s.feedback_hz,
                        'control': 1 / s.control_hz, 'actuator': 1 / s.actuator_hz}
        self.next_due = self.periods.copy()
        self.last_run = dict.fromkeys(self.periods, 0.0)
        self.counts = dict.fromkeys(self.periods, 0)
        self.feedback_received = 0
        self.control_intervals = []
        self.physical_peak = self.local_sensor_peak = 0.0
        self.physics_peak = 0.0
        self.brake_started = None
        self.terminal = 'STOPPED'
        self.stop_confirmed = False
        self.local_stop_confirmed = False
        self._last_phase = self.controller.phase

    @property
    def sensor(self):
        return self.latest.sensor

    @property
    def stopped(self):
        return self.controller.phase in ('STOPPED', 'FINISHED')

    @property
    def paused(self):
        return self.controller.phase == 'PAUSED'

    def due(self, name):
        if self.time + 1e-10 < self.next_due[name]:
            return None
        dt = self.time - self.last_run[name]
        self.last_run[name] = self.time
        self.next_due[name] += self.periods[name]
        self.counts[name] += 1
        return dt

    def request_stop(self, reason, terminal='STOPPED'):
        if self.brake_started is not None:
            if terminal == 'STOPPED':
                self.terminal = terminal
                self.controller.reason = reason
            return
        if self.paused:
            return
        self.controller.phase, self.controller.reason = 'BRAKING', reason
        self.controller.feed = self.controller.active_target = 0.0
        self.controller.normal_speed = self.controller.integral = 0.0
        self.terminal, self.brake_started = terminal, self.time
        self.driver.brake(self.time, self.physics)
        self.command = self.driver.command
        self.events.append({'time_s': self.time, 'event': 'brake_requested', 'reason': reason})

    def pause(self):
        self.request_stop('Operator pause: motion and pressing released.', 'PAUSED')

    def resume(self):
        if not self.paused:
            return
        self.brake_started = None
        self.stop_confirmed = False
        self.local_stop_confirmed = False
        self.driver.resume(self.time, self.physics)
        self.controller.transition('SETTLE' if self.sensor.raw.length > .3 else 'SEARCH', self.latest.position)
        self.controller.reason = ''
        self.events.append({'time_s': self.time, 'event': 'resumed'})

    def step(self):
        if self.stopped or self.paused:
            return
        s, p, c = self.settings, self.physics, self.controller
        dt = 1 / s.physics_hz
        self.physics.step(self.driver.command, dt)
        self.physics_peak = max(self.physics_peak, self.physics.step_force.length)
        self.steps += 1
        self.time = self.steps * dt
        sample_dt = self.due('sensor')
        if sample_dt is not None:
            self.produced = self.sensor_model.sample(self.time, sample_dt, p)
            self.physical_peak = max(self.physical_peak, p.sensor.raw.length)
            self.local_sensor_peak = max(self.local_sensor_peak, self.produced.sensor.raw.length)
            # An emulated onboard sensor interlock, not perfect collision-ID knowledge.
            if self.produced.ready and self.produced.sensor.raw.length > s.max_contact_force:
                self.request_stop(f'Local measured force exceeded {s.max_contact_force:g} N.')
        if self.due('feedback') is not None:
            self.feedback.send(self.time, self.produced, s.feedback_delay, s.jitter, s.dropout)
        state = self.feedback.receive(self.time)
        if state is not None:
            self.latest = state
            self.feedback_received += 1
        control_dt = self.due('control')
        if control_dt is not None:
            self.control_intervals.append(control_dt)
            c.force = self.sensor.filtered.dot(c.normal)
            if self.brake_started is None:
                if self.time - self.latest.sampled_at > s.state_timeout:
                    self.request_stop('Measurement age exceeded the feedback timeout.')
                elif self.latest.position.x >= self.terrain.end_x:
                    self.request_stop('End of the profile reached.', 'FINISHED')
                elif not (-.04 < self.latest.position.y < .6):
                    self.request_stop('Measured pose left the workspace.')
                else:
                    self.command = c.update(self.latest, control_dt)
                    if c.phase == 'STOPPED':
                        self.request_stop(c.reason)
                    else:
                        self.driver.submit(self.time, self.command)
            self.record(control_dt)
        actuator_dt = self.due('actuator')
        if actuator_dt is not None:
            self.driver.update(self.time, actuator_dt, self.physics)
            if self.driver.expired and self.brake_started is None:
                self.request_stop('Command watchdog expired.')
        if self.brake_started is not None:
            elapsed = self.time - self.brake_started
            self.local_stop_confirmed = (elapsed > .12 and p.body.velocity.length < .0008
                                         and abs(p.body.angular_velocity) < .01
                                         and self.produced.sensor.raw.length < .45)
            remote_confirmation = (self.latest.sampled_at > self.brake_started + .1
                                   and self.time - self.latest.sampled_at <= s.state_timeout
                                   and self.latest.velocity.length < .0008
                                   and abs(self.latest.angular_velocity) < .01
                                   and self.latest.sensor.raw.length < .45)
            if self.local_stop_confirmed and remote_confirmation:
                c.phase = self.terminal
                self.stop_confirmed = True
                self.events.append({'time_s': self.time, 'event': 'stop_confirmed'})
                self.record(0.0)
            elif elapsed > 1.0:
                c.phase, c.reason = 'STOPPED', 'Stop motion / load not confirmed within 1 s; simulation halted (stop unconfirmed).'
                self.events.append({'time_s': self.time, 'event': 'stop_unconfirmed'})
                self.record(0.0)

    def record(self, control_dt):
        p, c, s = self.physics, self.controller, self.settings
        settings = asdict(s)
        changes = {key: value for key, value in settings.items() if value != self.last_settings[key]}
        if changes:
            self.parameter_changes.append({'time_s': self.time, 'changes': changes})
            self.last_settings = settings
        if c.phase != self._last_phase:
            self.events.append({'time_s': self.time, 'event': c.phase})
            self._last_phase = c.phase
        _, truth_normal = self.terrain.nearest(p.body.position)
        contact = self.latest.position - s.tip_radius * c.normal
        if (c.phase == 'TRACK' and c.supervisor.risk < .5 and abs(c.force - s.target_force) < .35
                and (not self.points or (contact - self.points[-1]).length >= s.point_spacing)):
            self.points.append(contact)
            if p.sensor.contact is not None:
                self.true_points.append(p.sensor.contact)
        row = dict(time_s=self.time, phase=c.phase, x_m=p.body.position.x, y_m=p.body.position.y,
                   measured_x_m=self.latest.position.x, measured_y_m=self.latest.position.y,
                   vx_mps=p.body.velocity.x, vy_mps=p.body.velocity.y,
                   fx_n=self.sensor.filtered.x, fy_n=self.sensor.filtered.y,
                   raw_fx_n=self.sensor.raw.x, raw_fy_n=self.sensor.raw.y,
                   true_fx_n=p.sensor.raw.x, true_fy_n=p.sensor.raw.y,
                   true_fn_n=p.sensor.raw.dot(truth_normal),
                   fn_n=c.force, target_n=s.target_force, active_target_n=c.active_target,
                   nx=c.normal.x, ny=c.normal.y, true_nx=truth_normal.x, true_ny=truth_normal.y,
                   tool_deg=degrees(p.body.angle), normal_deg=degrees(normal_angle(c.normal)),
                   true_normal_deg=degrees(normal_angle(truth_normal)),
                   alignment_deg=degrees(abs(angle_error(normal_angle(c.normal), p.body.angle))),
                   normal_error_deg=degrees(abs(angle_error(normal_angle(truth_normal), normal_angle(c.normal)))),
                   command_vx_mps=self.command.velocity.x, command_vy_mps=self.command.velocity.y,
                   applied_vx_mps=self.driver.command.velocity.x, applied_vy_mps=self.driver.command.velocity.y,
                   feed_mps=c.feed, actual_tangent_mps=p.body.velocity.dot((c.normal.y, -c.normal.x)),
                   bounded_feed_mps=c.speed_limit, moment_residual_nm=c.moment_residual,
                   contact_x_m=contact.x, contact_y_m=contact.y, moment_nm=self.sensor.torque,
                   true_contact_x_m=p.sensor.contact.x if p.sensor.contact is not None else None,
                   true_contact_y_m=p.sensor.contact.y if p.sensor.contact is not None else None,
                   mu_est=c.observer.mu, shaft_force_n=p.sensor.shaft_force,
                   source_time_s=self.latest.sampled_at, state_age_ms=(self.time - self.latest.sampled_at) * 1000,
                   control_dt_ms=control_dt * 1000, fast_force_n=c.supervisor.fast,
                   predicted_force_n=c.supervisor.predicted, overload_risk=c.supervisor.risk,
                   unload_events=c.unload_events, normal_saturated=c.normal_saturated,
                   physics_friction=s.friction, sensor_noise_n=s.noise,
                   hybrid_kp=s.hybrid_kp, hybrid_ki=s.hybrid_ki,
                   admittance_mass_kg=s.admittance_mass, admittance_damping_ns_m=s.admittance_damping,
                   normal_reference_error_m=(self.command.position - self.latest.position).dot(c.normal))
        self.rows.append(row)
        self.history.append(row)

    def advance(self, seconds):
        for _ in range(round(seconds * self.settings.physics_hz)):
            self.step()
            if self.stopped or self.paused:
                break
