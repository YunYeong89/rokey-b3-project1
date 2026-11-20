"""MuJoCo run of the stand driven by the unchanged firmware (firmware/car_turn_lift.py): its lgpio calls go to SimGPIO
(TB6612 pins -> wheel actuators: drive, short brake, coast; servo pulse -> lift target) and its time.sleep steps the
physics (SimClock), so turns, lift moves, the 'c' calibration, stations and the --serve TCP protocol run as on the
Raspberry Pi, while the simulator reads the true pose the open-loop firmware cannot. Probe forces act on the vehicle as
external forces (no robot here). macOS viewer: mjpython -m turnlift mujoco --gui."""
import importlib.util
from pathlib import Path
import time
from types import SimpleNamespace

import mujoco
import numpy as np

from .config import OUT, ROOT
from .device import pose
from .mjcf import MjcfExport
from .motors import Servo


def wrap(deg):
    """-180 < deg <= 180 (the firmware turns +180 for exactly opposite)."""
    return 180 - (180 - deg) % 360


class SimGPIO:
    """lgpio stand-in: pin duty 0..100 (a written level is 0 or 100) and servo pulse widths (0 = off)."""
    error = OSError

    def __init__(self):
        self.pins, self.servo = {}, {}

    def gpiochip_open(self, chip):
        return 0

    def gpio_claim_output(self, h, pin, level=0):
        self.pins[pin] = 100. * level

    def gpio_write(self, h, pin, level):
        self.pins[pin] = 100. * level

    def tx_pwm(self, h, pin, freq, duty):
        self.pins[pin] = float(duty)

    def tx_servo(self, h, pin, width, freq=50):
        self.servo[pin] = width

    def gpiochip_close(self, h):
        pass

    def side(self, pins):
        """(duty -1..1, braked) of a motor channel: TB6612 EN / IN1 / IN2 (H H short brake, L L coast, PWM low = short
        brake) or a two-pin a / b driver."""
        level = lambda pin: self.pins.get(pin, 0.)
        if 'en' in pins:
            a, b = level(pins['in1']) > 50, level(pins['in2']) > 50
            return (0., a) if a == b else ((1 if a else -1) * level(pins['en']) / 100, True)
        a, b = level(pins['a']), level(pins['b'])
        return (0., a > 50) if a == b else ((a - b) / 100, True)

    def angle(self, pin):
        width = self.servo.get(pin, 0)
        return (width - 500) * 180 / 2000 if width else None


class SimClock:
    """`time` stand-in for the firmware: sleep runs the physics, monotonic is the simulated time."""

    def __init__(self, sim):
        self.sim = sim

    def sleep(self, seconds):
        self.sim.run(seconds)

    def monotonic(self):
        return self.sim.data.time


class TurnLiftSim:
    def __init__(self, device, gui=False):
        self.d, c = device, device.c
        self.folder = OUT / 'mujoco'
        self.model = mujoco.MjModel.from_xml_path(str(MjcfExport(device).write(self.folder)))
        self.data, m = mujoco.MjData(self.model), self.model
        self.gpio, self.servo = SimGPIO(), Servo(c['motors']['servo'])
        self.fw = self.firmware()
        self.wheels = {side: [m.actuator(f'wheel_{end}_{side}').id for end in ('front', 'rear')]
                       for side in ('left', 'right')}
        self.brake_damping = m.actuator_biasprm[self.wheels['left'][0], 2]
        self.lift = m.actuator('lift').id
        joint = m.joint('lift').id
        self.lift_q, self.lift_v = m.jnt_qposadr[joint], m.jnt_dofadr[joint]
        self.body, self.top = m.body('turn_lift').id, m.body('lift_top').id
        lift = device.lift
        angles = lift.angles(561)                                   # servo down -> up, the top rising
        tops = np.array([lift.top(a) for a in angles])
        self.base_top = tops[0]
        self.curve = (tops - tops[0]) / 1000, angles, np.array([lift.ratio(a) for a in angles]) / 1000  # m, deg, m/rad
        self.load, self.saturated = None, 0
        mujoco.mj_forward(m, self.data)
        self.yaw, self.last = np.radians(c['placement']['heading0_yaw_deg']), self.raw_yaw()
        self.viewer = None
        if gui:
            from mujoco import viewer
            self.viewer = viewer.launch_passive(m, self.data)
            self.viewer.cam.lookat[:], self.viewer.cam.distance, self.viewer.cam.elevation = [.5, 0, .15], 1.4, -30

    def firmware(self):
        """The firmware module with its GPIO, clock and state files replaced (fresh state files under out/mujoco)."""
        spec = importlib.util.spec_from_file_location('car_turn_lift_sim', ROOT / 'firmware/car_turn_lift.py')
        fw = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(fw)
        fw.lgpio, fw.time, fw.SIM = self.gpio, SimClock(self), False
        fw.LIFT_STATE, fw.CALIB_FILE = str(self.folder / 'lift_state.json'), str(self.folder / 'turn_calib.json')
        for path in (fw.LIFT_STATE, fw.CALIB_FILE):
            Path(path).unlink(missing_ok=True)
        return fw

    # ---------------- physics ----------------
    def step(self):
        m, d = self.model, self.data
        for side, pins in (('left', self.fw.LEFT), ('right', self.fw.RIGHT)):
            duty, braked = self.gpio.side(pins)
            for i in self.wheels[side]:
                d.ctrl[i] = duty
                m.actuator_biasprm[i, 2] = self.brake_damping if braked else 0.   # coasting: no back-EMF current
        angle = self.gpio.angle(self.fw.LIFT_PIN)
        ratio = np.interp(d.qpos[self.lift_q], self.curve[0], self.curve[2])
        limit = 0. if angle is None else self.servo.torque(d.qvel[self.lift_v] / ratio) * self.servo.efficiency / ratio
        m.actuator_forcerange[self.lift] = -limit, limit                       # MG996R through the scissor, here
        if angle is not None:
            d.ctrl[self.lift] = np.interp(angle, self.curve[1][::-1], self.curve[0][::-1])
        d.qfrc_applied[:] = 0
        if self.load is not None:
            local, force, start, ramp = self.load
            mujoco.mj_applyFT(m, d, force * min(1., (d.time - start) / ramp), np.zeros(3), self.load_point() / 1000,
                              self.top, d.qfrc_applied)
        mujoco.mj_step(m, d)
        self.saturated += limit > 0 and abs(d.actuator_force[self.lift]) >= .99 * limit
        yaw = self.raw_yaw()
        self.yaw += (yaw - self.last + np.pi) % (2 * np.pi) - np.pi
        self.last = yaw

    def run(self, seconds):
        wall = time.perf_counter() - self.data.time
        for i in range(max(1, round(seconds / self.model.opt.timestep))):
            self.step()
            if self.viewer is not None and i % 16 == 0:
                if not self.viewer.is_running():
                    raise KeyboardInterrupt
                self.viewer.sync()
                time.sleep(max(0., self.data.time - (time.perf_counter() - wall)))

    def push(self, point_mm, force_n, ramp=.3):
        """Probe force (N, BASE) at a vehicle point (BASE mm), ramped in over `ramp` s and moving with the vehicle;
        None clears it."""
        if point_mm is None:
            self.load = None
            return
        d = self.data
        local = d.xmat[self.top].reshape(3, 3).T @ (np.asarray(point_mm) / 1000 - d.xpos[self.top])
        self.load = local, np.asarray(force_n, float), d.time, ramp

    def place(self, centre_mm, heading):
        """Sets the car at rest on a BASE turning centre (mm) and true firmware heading (deg), the lift as it is."""
        m, d = self.model, self.data
        q = m.jnt_qposadr[m.joint('turn_lift').id]
        yaw = np.radians(self.d.c['placement']['heading0_yaw_deg'] - heading)
        d.qpos[q:q + 7] = [*np.asarray(centre_mm) / 1000, d.qpos[q + 2], np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)]
        d.qvel[:] = 0
        mujoco.mj_forward(m, d)
        self.yaw, self.last = yaw, self.raw_yaw()

    def save(self):
        d = self.data
        return d.qpos.copy(), d.qvel.copy(), d.time, self.yaw, self.last

    def restore(self, state):
        d = self.data
        d.qpos[:], d.qvel[:], d.time, self.yaw, self.last = state
        mujoco.mj_forward(self.model, d)

    def close(self):
        if self.viewer is not None:
            self.viewer.close()

    # ---------------- truth ----------------
    def raw_yaw(self):
        r = self.data.xmat[self.body]
        return np.arctan2(r[3], r[0])

    def heading(self):
        """True firmware heading (deg, clockwise seen from above, unwrapped)."""
        return self.d.c['placement']['heading0_yaw_deg'] - np.degrees(self.yaw)

    def centre(self):
        return self.data.xpos[self.body][:2] * 1000

    def tilt(self):
        return np.degrees(np.arccos(np.clip(self.data.xmat[self.body][8], -1, 1)))

    def top_mm(self):
        return self.base_top + self.data.qpos[self.lift_q] * 1000

    def load_point(self):
        d = self.data
        return (d.xpos[self.top] + d.xmat[self.top].reshape(3, 3) @ self.load[0]) * 1000

    def vehicle_points(self):
        """Vehicle surface points (BASE mm) at the simulated pose."""
        d, yaw = self.data, self.d.c['placement']['vehicle_yaw_on_car_deg']
        local = self.d.vehicle.triangles.reshape(-1, 3)[::5] @ pose(0, 0, 0, yaw)[:3, :3].T
        return d.xpos[self.top] * 1000 + local @ d.xmat[self.top].reshape(3, 3).T

    def truth(self):
        return dict(heading=round(float(self.heading()) % 360, 1), centre_mm=self.centre().round(1).tolist(),
                    top_mm=round(float(self.top_mm()), 1), tilt_deg=round(float(self.tilt()), 2))


class Scenario:
    """Firmware commands on the simulated stand: what the open-loop firmware believes against the simulated truth."""

    def __init__(self, sim):
        self.sim, self.fw = sim, sim.fw
        self.car = sim.fw.Car()
        sim.run(1.)                                                             # settle on the wheels

    def reset(self, station):
        """True pose = what the firmware believes (the station's centre, heading 0), so each test stands alone."""
        self.sim.place(self.sim.d.c['scan']['directions'][station]['centre_mm'], 0.)
        self.car.heading, self.car.station = 0., station
        self.sim.run(.3)

    def calibrate(self, tries=12):
        """The firmware's 'c' procedure until 85-95 deg, the simulator reading the turned angle for the user."""
        rows, keys = [], SimpleNamespace(normal=lambda: None, raw=lambda: None)
        for _ in range(tries):
            start, duty = self.sim.heading(), self.car.duty
            self.fw.input = lambda _prompt: f'{self.sim.heading() - start:.1f}'
            self.car.calibrate(keys)
            rows.append(dict(duty_pct=duty, turned_deg=self.sim.heading() - start))
            if 85 <= rows[-1]['turned_deg'] <= 95:
                break
        return rows

    def turns(self, targets=(90, 180, 270, 0)):
        rows = []
        for target in targets:
            start, commanded = self.sim.heading(), wrap(target - self.car.heading)
            self.car.turn_to(target)
            self.sim.run(.5)
            rows.append(dict(station=self.car.station, top_mm=self.sim.top_mm(), target=target, commanded_deg=commanded,
                             turned_deg=self.sim.heading() - start, error_deg=wrap(self.sim.heading() - target),
                             centre_mm=self.sim.centre().round(1).tolist()))
        return rows

    def lift(self, up):
        angle = self.fw.UP_ANGLE if up else self.fw.DOWN_ANGLE
        start, self.sim.saturated = self.sim.data.time, 0
        self.car.lift_to(angle)
        moving = self.sim.data.time - start
        self.sim.run(1.)
        target = self.sim.d.lift.top(angle)
        return dict(to='up' if up else 'down', target_mm=target, actual_mm=self.sim.top_mm(),
                    error_mm=self.sim.top_mm() - target, move_s=moving,
                    force_limited_s=self.sim.saturated * self.sim.model.opt.timestep)

    def station(self, name):
        start = self.sim.centre()
        self.car.station_to(name)
        self.sim.run(.5)
        travel = self.sim.centre() - start
        return dict(to=name, believed_x_mm=self.fw.STATION_MM * (1 if name == 'front' else -1), travel_x_mm=travel[0],
                    drift_y_mm=travel[1], heading_error_deg=wrap(self.sim.heading() - self.car.heading))

    def hold(self, load, force, brake=True, seconds=2.):
        """Probe force on the standing car: `front` along BASE +x at the vehicle point nearest the robot, `top` along
        -z at the high point farthest from the turning centre (as loads.Holding). The state is restored after."""
        sim = self.sim
        state, points = sim.save(), sim.vehicle_points()
        if load == 'front':
            point, direction = points[np.argmin(points[:, 0])], np.array([1., 0, 0])
        else:
            high = points[points[:, 2] > points[:, 2].max() - 30]
            point, direction = high[np.argmax(np.linalg.norm(high[:, :2] - sim.centre(), axis=1))], np.array([0, 0, -1.])
        if not brake:
            self.car.stop()
        rise, centre, heading, tilt = sim.data.qpos[sim.lift_q], sim.centre(), sim.heading(), 0.
        sim.push(point, force * direction)
        for _ in range(int(seconds / .05)):
            sim.run(.05)
            tilt = max(tilt, sim.tilt())
        row = dict(load=load, force_n=force, brake=brake, shift_mm=(sim.load_point() - point) @ direction,
                   slide_mm=np.linalg.norm(sim.centre() - centre), yaw_deg=sim.heading() - heading,
                   tilt_max_deg=tilt, sink_mm=(rise - sim.data.qpos[sim.lift_q]) * 1000, tipped=bool(tilt > 15))
        sim.push(None, None)
        sim.restore(state)
        self.car.brake()
        return row

    def run(self):
        """Calibration, then each test from the pose the firmware believes: four turns at the top station (lift
        down), the drive to the front station, lift up, four turns there, holding tests, lift down, the drive back."""
        result = dict(calibration=self.calibrate(), calibrated=[dict(duty_pct=self.car.duty, time_90_s=self.car.time_90)])
        self.reset('top')
        result['turns'] = self.turns()
        self.reset('top')
        result['station'] = [self.station('front')]
        self.reset('front')
        result['lift'] = [self.lift(True)]
        result['turns'] += self.turns()
        self.reset('front')
        result['hold'] = [self.hold('front', 1.5, brake=False)] + [self.hold('front', f) for f in (1.5, 3., 8.)] + \
                         [self.hold('top', f) for f in (1.5, 5., 8.)]
        result['lift'].append(self.lift(False))
        result['station'].append(self.station('top'))
        self.car.close()
        return result

    def serve(self, port):
        """The firmware TCP protocol (heading / turn / lift / station / drive / status / quit) on the simulated stand
        after the calibration; each reply also carries `sim` = the true pose. The physics runs only during commands."""
        calibration = self.calibrate()
        self.reset('top')
        command = self.fw.command
        self.fw.command = lambda car, words, table: {**command(car, words, table), 'sim': self.sim.truth()}
        try:
            self.fw.serve(self.car, port)
        except KeyboardInterrupt:
            pass
        finally:
            self.car.close()
        return dict(calibration=calibration, port=port, final=self.sim.truth())
