"""Virtual-only turntable stand (repair_car native demo_final): the scaled lift (device.yaml `linkage`) on a turntable disc
(`turntable`) on the table, the vehicle resting on the lift's holder face. The disc turns the lift and the vehicle about
its centre (no drive is modelled); the crank follows the firmware servo angle (`servo_down_up_deg`, lift_table.json).
Pose maths only (numpy): the native session places the meshes in Gazebo and the viewers.

Commands follow the firmware TCP grammar (firmware/car_turn_lift.py `command`), served as custom_interfaces LiftCommand:
  heading DEG | turn DEG | lift up | lift down | lift MM (holder top above the lift base bottom) | status
heading is clockwise seen from above, 0 = vehicle front toward the robot (BASE yaw = placement.heading0_yaw_deg - heading).
Units mm, deg; poses are 4x4 in BASE (mm)."""
import numpy as np

from .assembly import LiftAssembly
from .config import ROOT, load
from .mesh import write_stl

LINKS = ('base', 'crank', 'pushrod', 'slider_bottom', 'arm_b', 'arm_a', 'slider_top', 'top')


def xz_rotation(angle):
    """Rotation in the lift model X-Z plane (pins along Y), + from X toward Z."""
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, 0, -s], [0, 1, 0], [s, 0, c]])


def frame(position, rotation=np.eye(3)):
    matrix = np.eye(4)
    matrix[:3, :3], matrix[:3, 3] = rotation, position
    return matrix


def yaw_frame(x, y, z, yaw_deg):
    c, s = np.cos(np.radians(yaw_deg)), np.sin(np.radians(yaw_deg))
    return frame([x, y, z], np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]))


def direction(a, b):
    return np.arctan2(b[1] - a[1], b[0] - a[0])


class VirtualStand:
    def __init__(self, config=None):
        self.c = config or load()
        self.lift = LiftAssembly(self.c['linkage'], self.c['lift'], ROOT)
        self.down, self.up = self.c['linkage']['servo_down_up_deg']
        table = self.lift.table(self.down, self.up)
        self.servo_table, self.top_table = np.array(table['servo_deg']), np.array(table['top_mm'])
        disc = self.c['turntable']
        self.disc_mm, self.disc_radius, self.disc_colour = disc['thickness_mm'], disc['diameter_mm'] / 2, disc['colour']
        self.heading0 = self.c['placement']['heading0_yaw_deg']
        self.face = self.lift.origins['top'] + self.lift.holder   # holder face centre, lift model frame, lift down
        self.centre, self.heading, self.servo = np.zeros(2), 0., float(self.down)

    # ---------------- lift height ----------------
    def top_mm(self, servo):
        """Holder top above the lift base bottom (lift_table.json)."""
        return float(np.interp(servo, self.servo_table[::-1], self.top_table[::-1]))

    def servo_for(self, top_mm):
        if not self.top_table[0] - 1e-6 <= top_mm <= self.top_table[-1] + 1e-6:
            raise ValueError(f'lift {top_mm:.2f} mm is outside {self.top_table[0]:.1f}..{self.top_table[-1]:.1f} mm')
        return float(np.interp(top_mm, self.top_table, self.servo_table))

    def stand_top(self):
        """Holder top above the table: the turntable plus the lift."""
        return self.disc_mm + self.top_mm(self.servo)

    def seat(self, vehicle_triangles):
        """Vehicle frame origin (wheel bottoms) above the holder face when the vehicle rests on it (mm, negative: the
        underside rests on the face, the wheels hang below it)."""
        return self.lift.rest(vehicle_triangles)

    # ---------------- placement ----------------
    def yaw(self):
        return self.heading0 - self.heading

    def place_vehicle(self, x, y, yaw_deg, base_z, table_z, seat):
        """Turn and raise the stand so that the vehicle frame is at [x, y, yaw] and base_z (BASE mm); the disc centre
        follows the vehicle (the holder face centre sits over the lift base centre)."""
        servo = self.servo_for(base_z - table_z - seat - self.disc_mm)
        self.heading, self.servo = (self.heading0 - yaw_deg) % 360., servo
        offset = yaw_frame(0, 0, 0, yaw_deg)[:2, :2] @ self.face[:2]
        self.centre = np.array([x, y], float) - offset

    def vehicle_pose(self, table_z, seat):
        """([x, y, yaw] BASE mm deg, base_z mm) of the vehicle resting on the stand."""
        yaw = self.yaw()
        xy = self.centre + yaw_frame(0, 0, 0, yaw)[:2, :2] @ self.face[:2]
        yaw = 180. - (180. - yaw) % 360.   # (-180, 180], as the repair_car placements
        return [round(float(xy[0]), 3), round(float(xy[1]), 3), round(float(yaw), 3)], \
            round(float(table_z + self.stand_top() + seat), 3)

    def command(self, line):
        """One firmware command line -> the firmware-style reply dict; the stand state changes on success."""
        words = line.split()
        try:
            if words[0] == 'heading':
                self.heading = float(words[1]) % 360.
            elif words[0] == 'turn':
                self.heading = (self.heading + float(words[1])) % 360.
            elif words[0] == 'lift':
                self.servo = float(self.up if words[1] == 'up' else self.down if words[1] == 'down'
                                   else self.servo_for(float(words[1])))
            elif words[0] in ('station', 'drive'):
                raise ValueError(f'{words[0]}: the virtual stand turns in place and has no car to drive')
            elif words[0] != 'status':
                raise ValueError(f'Unknown command: {words[0]}')
        except (IndexError, ValueError) as error:
            return {'ok': False, 'error': str(error) or 'An argument is required'}
        return self.status()

    def status(self):
        return {'ok': True, 'heading': round(self.heading, 1), 'station': 'top', 'lift_deg': round(self.servo, 1),
                'lift_mm': round(self.top_mm(self.servo), 2), 'stand_top_mm': round(self.stand_top(), 2)}

    # ---------------- meshes and poses ----------------
    def link_frames(self, servo=None):
        """Lift link frames (4x4, lift model frame mm) at a servo angle: each link's mesh is about its origin."""
        a, angle = self.lift, np.radians(abs((self.servo if servo is None else servo) - self.down))
        axes, origins = a.axes, a.origins
        c, f, s0, t0 = axes['crank'], axes['fixed_bottom'], axes['slider_bottom'], axes['slider_top']
        tip = c + a.radius * np.array([np.cos(a.phase + angle), np.sin(a.phase + angle)])
        slider = np.array([a.slider_x(angle), s0[1]])
        rise = a.rise(slider[0])
        top_slider, fixed_top = np.array([slider[0], t0[1] + rise]), axes['fixed_top'] + [0, rise]
        point = lambda xz: np.array([xz[0], 0., xz[1]])
        turn = lambda now, before: xz_rotation(now - before)
        return dict(
            base=np.eye(4),
            crank=frame(origins['crank'], xz_rotation(angle)),
            pushrod=frame(point(tip), turn(direction(tip, slider), direction(axes['crank_pin'], s0))),
            slider_bottom=frame(point(slider)),
            arm_b=frame(point(slider), turn(direction(slider, fixed_top), direction(s0, axes['fixed_top']))),
            arm_a=frame(point((f + top_slider) / 2), turn(direction(f, top_slider), direction(f, t0))),
            slider_top=frame(point(top_slider)),
            top=frame(origins['top'] + [0, 0, rise]))

    def meshes(self, directory):
        """Binary STL meshes (mm, about each link origin; the disc about its bottom centre) -> {name: path}."""
        paths = {}
        for name in LINKS:
            paths[name] = directory / f'stand_{name}.stl'
            write_stl(paths[name], self.lift.links[name]['triangles'])
        paths['disc'] = directory / 'stand_disc.stl'
        write_stl(paths['disc'], self.disc())
        return paths

    def disc(self, sides=64):
        a = np.linspace(0, 2 * np.pi, sides, endpoint=False)
        ring = np.c_[np.cos(a), np.sin(a)] * self.disc_radius
        low, high = np.c_[ring, np.zeros(sides)], np.c_[ring, np.full(sides, self.disc_mm)]
        n, centre = np.roll(np.arange(sides), -1), np.zeros(3)
        top, bottom = [0, 0, self.disc_mm], centre
        triangles = [*(np.stack([low[i], low[n[i]], high[n[i]]]) for i in range(sides)),
                     *(np.stack([low[i], high[n[i]], high[i]]) for i in range(sides)),
                     *(np.stack([top, high[i], high[n[i]]]) for i in range(sides)),
                     *(np.stack([bottom, low[n[i]], low[i]]) for i in range(sides))]
        return np.array(triangles, float)

    def frames(self, table_z):
        """BASE frames (4x4 mm) of the disc and the lift links at the current heading and servo angle."""
        stand = yaw_frame(*self.centre, table_z, self.yaw())
        lift = stand @ frame([0, 0, self.disc_mm])
        return {'disc': stand, **{name: lift @ link for name, link in self.link_frames().items()}}
