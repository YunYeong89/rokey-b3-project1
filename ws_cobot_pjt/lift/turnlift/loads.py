"""Load checks of the stand with the vehicle: lift servo torque, in-place turn on the TT motors, holding against the
probe force (tipping, sliding, yawing) and the top plate's play. Quasi-static; SI internally, reported in N, N m,
kg cm, mm."""
import numpy as np

from .device import G, apply
from .motors import KGCM, GearMotor, Servo


class LiftTorque:
    """Virtual work: servo torque = (lifted weight + downward probe force) x d(top)/d(servo angle) / efficiency.
    Arms and the halves of rods / spacers / clips rise at half the top's rate (scissor.py `rise`)."""

    def __init__(self, device):
        self.d, self.servo = device, Servo(device.c['motors']['servo'])
        self.lifted = device.lift.rising + device.adapter_mass + device.vehicle.mass

    def curve(self, force_n=0.):
        lift = self.d.lift
        return [dict(servo_deg=a, top_mm=lift.top(a), ratio_mm_rad=lift.ratio(a),
                     torque_nm=(self.lifted * G + force_n) * lift.ratio(a) / 1000 / self.servo.efficiency)
                for a in lift.angles(71)]

    def check(self, force_n):
        rows = self.curve(force_n)
        worst = max(rows, key=lambda r: r['torque_nm'])
        available = self.servo.usable * self.servo.torque(self.servo.speed)
        return dict(lifted_kg=self.lifted, force_n=force_n, worst_servo_deg=worst['servo_deg'],
                    required_kgcm=worst['torque_nm'] / KGCM, usable_kgcm=available / KGCM,
                    stall_kgcm=self.servo.stall / KGCM, margin=available / worst['torque_nm'])


class Support:
    """Wheel contacts, loads and friction of the car (car frame)."""

    def __init__(self, device, servo):
        car = device.c['car']
        self.car, self.r = car, car['wheel_radius_mm'] / 1000
        self.mass, self.cog = device.total(servo)
        L, T = car['wheelbase_mm'] / 2, car['track_mm'] / 2
        self.wheels = np.array([[sx * L, sy * T] for sx in (1, -1) for sy in (1, -1)])
        x, y = self.cog[:2]
        self.loads = self.mass * G / 4 * (1 + x * self.wheels[:, 0] / L ** 2 + y * self.wheels[:, 1] / T ** 2)


class TurnTorque:
    """In-place skid-steer turn: lateral tyre scrub mu_lat N_i at the wheels' longitudinal lever plus rolling
    resistance, balanced by the left/right traction couple over the track; the wheel motors at the firmware's speed
    (90 deg in TIME_90)."""

    def __init__(self, device):
        self.d, self.motor = device, GearMotor(device.c['motors']['wheel'])

    def check(self, servo):
        s, f = Support(self.d, servo), self.d.c['car']['friction']
        resist = (f['lateral'] * s.loads @ np.abs(s.wheels[:, 0]) + f['rolling'] * s.loads @ np.abs(s.wheels[:, 1])) / 1000
        track = self.d.c['car']['track_mm'] / 1000
        wheel_force = resist / track / 2                     # each of the two wheels per side
        torque = wheel_force * s.r
        turn_rate = np.radians(90) / self.d.c['motors']['wheel']['time_90_s']
        wheel_speed = turn_rate * track / 2 / s.r
        duty = self.motor.duty_for(torque, wheel_speed)
        return dict(servo_deg=servo, mass_kg=s.mass, cog_mm=s.cog.round(1).tolist(), resist_nm=resist,
                    wheel_torque_kgcm=torque / KGCM, duty_needed_pct=100 * duty,
                    duty_firmware_pct=100 * self.motor.duty, traction=f['longitudinal'] * s.loads.min() / wheel_force)


class Holding:
    """A probe force on the vehicle with the car standing: the centre of pressure must stay inside the wheel contacts
    (tipping), the horizontal force below what the tyres / gearboxes hold (coasting or TB6612 short brake), and its
    moment about the turning centre below the scrub resistance (yawing)."""

    def __init__(self, device):
        self.d = device

    def check(self, heading, servo, point, force):
        """`point` BASE mm, `force` BASE N acting on the vehicle."""
        s, car = Support(self.d, servo), self.d.c['car']
        frame = self.d.car_pose(heading)
        p = apply(np.linalg.inv(frame), [point])[0] / 1000
        f = frame[:3, :3].T @ np.asarray(force, float)
        weight = np.r_[0., 0., -s.mass * G]
        loads = [(s.cog / 1000, weight), (p, f)]
        fz = sum(load[2] for _, load in loads)
        cop = np.array([sum(r[i] * load[2] - r[2] * load[i] for r, load in loads) / fz for i in (0, 1)]) * 1000
        L, T = car['wheelbase_mm'] / 2, car['track_mm'] / 2
        tip_margin = min(L - abs(cop[0]), T - abs(cop[1]))
        fr = car['friction']
        normal = -fz
        roll = len(s.wheels) * car['backdrive_nm'] / s.r + fr['rolling'] * normal   # coasting, along the car x
        yaw = abs(p[0] * f[1] - p[1] * f[0])
        scrub = (fr['lateral'] * s.loads @ np.abs(s.wheels[:, 0])) / 1000
        factor = lambda capacity, load: min(99., capacity / max(load, 1e-9))   # 99: no such load
        return dict(heading=heading, servo_deg=servo, cop_mm=cop.round(1).tolist(), tip_margin_mm=tip_margin,
                    slide_x_coast=factor(roll, abs(f[0])), slide_x_brake=factor(fr['longitudinal'] * normal, abs(f[0])),
                    slide_y=factor(fr['lateral'] * normal, abs(f[1])), yaw=factor(scrub, yaw))


class Play:
    """Top plate play from pin clearances: the load path base -> arm -> middle pin -> arm -> top pin -> plate has
    three pins (each radial play), the plate tilts by the slot / pin play over the pin span; a surface point h above
    the plate moves by the shift + tilt x h. A steady probe force takes the play up to one side: a bias per heading."""

    def __init__(self, device):
        self.d = device

    def check(self, servo, height_mm):
        lift, play = self.d.lift, self.d.c['lift']['pin_play_mm']
        span = lift.lc.rod_x(lift.crank(servo)) - lift.lc.XF
        tilt = 2 * play / span
        return dict(servo_deg=servo, shift_mm=3 * play, tilt_deg=np.degrees(tilt), at_height_mm=height_mm,
                    surface_mm=3 * play + tilt * height_mm)
