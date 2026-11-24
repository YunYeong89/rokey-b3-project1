"""The stand: car + scissor lift + adapter plate + vehicle. Frames: BASE (robot, table z = 0), car (x front, origin on
the table under the turning centre), lift (scissor.py), vehicle (vehicle.py). A firmware heading h (deg, clockwise
seen from above: key '6' = 90 = right turn) puts the car front at BASE yaw heading0_yaw - h."""
import numpy as np

from .config import ROOT
from .mesh import box_inertia, box_triangles, sample
from .scissor import ScissorLift
from .vehicle import Vehicle

G = 9.81


def pose(x, y, z, yaw_deg=0.):
    c, s = np.cos(np.radians(yaw_deg)), np.sin(np.radians(yaw_deg))
    transform = np.eye(4)
    transform[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]
    transform[:3, 3] = x, y, z
    return transform


def apply(transform, points):
    return np.asarray(points) @ transform[:3, :3].T + transform[:3, 3]


class TurnLift:
    def __init__(self, config, root=ROOT):
        self.c = config
        self.lift = ScissorLift(config['lift'], root)
        self.vehicle = Vehicle(config['vehicle'], root)
        a = config['adapter']
        fill = config['lift']['shell'] + (1 - config['lift']['shell']) * a['infill']
        self.adapter_mass = np.prod(a['size_mm']) * config['lift']['density_g_cm3'][a['material']] * fill * 1e-6

    # ---------------- frames ----------------
    def car_pose(self, heading, centre=None):
        """`centre`: BASE x, y of the turning centre (a station), placement centre_mm by default."""
        p = self.c['placement']
        return pose(*(p['centre_mm'] if centre is None else centre), 0., p['heading0_yaw_deg'] - heading)

    def deck(self):
        """Lift frame in the car frame (lift centre at mount_mm, on the car top)."""
        lift = self.c['lift']
        return pose(*lift['mount_mm'], self.c['car']['body_mm'][2], lift['yaw_on_car_deg']) \
            @ pose(-self.lift.size[0] / 2, 0, 0)

    def plate_z(self, servo):
        """Adapter top (the vehicle's ground) in the car frame."""
        return self.c['car']['body_mm'][2] + self.lift.top(servo) + self.c['adapter']['size_mm'][2]

    def vehicle_in_car(self, servo):
        return pose(*self.c['lift']['mount_mm'], self.plate_z(servo), self.c['placement']['vehicle_yaw_on_car_deg'])

    def vehicle_pose(self, heading, servo, centre=None):
        """Vehicle frame -> BASE."""
        return self.car_pose(heading, centre) @ self.vehicle_in_car(servo)

    # ---------------- bodies ----------------
    def fixed_body(self):
        """Car + the lift's non-rising parts as one simulated body: mass kg, centre m, inertia kg m2 (car frame)."""
        mass = self.c['car']['mass_kg'] + self.lift.mass - self.lift.rising
        return mass, np.r_[0., 0., self.c['car']['cog_z_mm']] / 1000, box_inertia(mass, self.c['car']['body_mm'])

    def moving_body(self):
        """Lift top + adapter + vehicle on the lift joint: mass kg, centre m, inertia kg m2 in the lift top frame
        (origin on the adapter top at the mount, i.e. the vehicle's ground)."""
        v = self.vehicle
        mass = self.lift.rising + self.adapter_mass + v.mass
        rz = pose(0, 0, 0, self.c['placement']['vehicle_yaw_on_car_deg'])[:3, :3]
        inertia = rz @ v.inertia @ rz.T * 1e-6 + box_inertia(mass - v.mass, self.c['adapter']['size_mm'])
        return mass, v.mass * (rz @ v.centre) / mass / 1000, inertia

    def boxes(self, servo):
        """Stand outline as boxes in the car frame: [(name, low, high)] (the scissor as its bounding box)."""
        car, lc = self.c['car'], self.lift.lc
        (L, W, H), r, half = car['body_mm'], car['wheel_radius_mm'], car['wheel_width_mm'] / 2
        result = [('car', np.array([-L / 2, -W / 2, r / 2]), np.array([L / 2, W / 2, H]))]
        for sx in (1, -1):
            for sy in (1, -1):
                x, y = sx * car['wheelbase_mm'] / 2, sy * car['track_mm'] / 2
                result.append((f'wheel{len(result)}', np.array([x - r, y - half, 0.]), np.array([x + r, y + half, 2 * r])))
        top = self.lift.top(servo)
        lift = [('lift_base', [0, -lc.BW / 2, 0], [lc.BL, lc.BW / 2, lc.WALL_H]),
                ('scissor', [0, -lc.BW / 2, lc.WALL_H], [lc.BL, lc.BW / 2, top - lc.TOP_T - lc.FLANGE_D]),
                ('lift_top', [0, -lc.BW / 2, top - lc.TOP_T - lc.FLANGE_D], [lc.BL, lc.BW / 2, top])]
        deck = self.deck()
        for name, low, high in lift:
            corners = apply(deck, np.array([low, high], float))
            result.append((name, corners.min(0), corners.max(0)))
        (al, aw, at), mount = self.c['adapter']['size_mm'], self.c['lift']['mount_mm']
        z = self.plate_z(servo)
        result.append(('adapter', np.array([mount[0] - al / 2, mount[1] - aw / 2, z - at]),
                       np.array([mount[0] + al / 2, mount[1] + aw / 2, z])))
        return result

    def masses(self, servo):
        """[(name, kg, centre in the car frame mm)] of the whole stand with the vehicle."""
        boxes = {name: (low + high) / 2 for name, low, high in self.boxes(servo)}
        lift = self.lift
        return [('car', self.c['car']['mass_kg'], np.r_[0., 0., self.c['car']['cog_z_mm']]),
                ('lift', lift.mass, (boxes['lift_base'] + boxes['lift_top']) / 2),
                ('adapter', self.adapter_mass, boxes['adapter']),
                ('vehicle', self.vehicle.mass, (self.vehicle_in_car(servo) @ np.r_[self.vehicle.centre, 1.])[:3])]

    def total(self, servo):
        rows = self.masses(servo)
        mass = sum(m for _, m, _ in rows)
        return mass, sum(m * c for _, m, c in rows) / mass

    def sweep(self, servo):
        """Largest horizontal distance from the turning centre of the stand and the vehicle (mm)."""
        corners = [np.array([[x, y] for x in (low[0], high[0]) for y in (low[1], high[1])]) for _, low, high in self.boxes(servo)]
        vehicle = apply(self.vehicle_in_car(servo), self.vehicle.triangles.reshape(-1, 3)[::7])[:, :2]
        return float(np.max(np.linalg.norm(np.concatenate([*corners, vehicle]), axis=1)))

    def obstacles(self, heading, servo, spacing, centre=None):
        """Stand surface points (boxes) in BASE."""
        points = [sample(box_triangles(low, high), spacing)[0] for _, low, high in self.boxes(servo)]
        return apply(self.car_pose(heading, centre), np.concatenate(points))
