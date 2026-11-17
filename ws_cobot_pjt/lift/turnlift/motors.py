"""Motor models from datasheet values: linear torque-speed lines. SI (N m, rad/s)."""
import numpy as np

KGCM = 0.0980665  # N m per kg cm


class Servo:
    """Hobby servo at the supply voltage (datasheet points interpolated): `stall` N m, `no_load` rad/s."""

    def __init__(self, spec):
        volts = sorted(spec['stall_kgcm'])
        self.stall = float(np.interp(spec['voltage'], volts, [spec['stall_kgcm'][v] for v in volts])) * KGCM
        seconds = float(np.interp(spec['voltage'], volts, [spec['speed_s_per_60deg'][v] for v in volts]))
        self.no_load = np.radians(60) / seconds
        self.usable, self.efficiency, self.speed = spec['usable'], spec['efficiency'], np.radians(spec['speed_deg_s'])

    def torque(self, speed=0.):
        return self.stall * max(0., 1 - abs(speed) / self.no_load)


class GearMotor:
    """Geared DC motor on PWM duty d (0..1): torque = d stall - stall / no_load x speed."""

    def __init__(self, spec):
        self.stall = spec['stall_kgcm'] * KGCM
        self.no_load = spec['no_load_rpm'] * 2 * np.pi / 60
        self.duty = spec['duty_pct'] / 100

    def torque(self, duty, speed):
        return max(0., duty * self.stall - self.stall / self.no_load * abs(speed))

    def duty_for(self, torque, speed):
        return torque / self.stall + abs(speed) / self.no_load
