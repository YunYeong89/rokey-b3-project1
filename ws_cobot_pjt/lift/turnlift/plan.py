"""Which vehicle surfaces the robot can probe with the stand: for every heading x lift state x direction (`top`:
pressing down along BASE -z, `front`: pressing along BASE +x on a face toward the robot, each at its station), the
vehicle surface outermost along that direction and tilted at most max_tilt from it, and of that what the probe reaches
with the tool and the arm clear (reach.py). Coverage per part over all states, on the vehicle's own surface samples."""
import numpy as np

from .device import apply
from .mesh import normals, sample
from .reach import Clearance, SimpleArm

PARTS = ('bumper', 'bonnet', 'door', 'body')


def outermost(points, direction, cell):
    """Indices of the points within 1 mm of the outermost along `direction` on a grid across it."""
    u = np.cross(direction, [0., 0., 1.]) if abs(direction[2]) < .9 else np.array([1., 0., 0.])
    u /= np.linalg.norm(u)
    v = np.cross(direction, u)
    keys = np.floor(np.c_[points @ u, points @ v] / cell).astype(np.int64)
    _, cells = np.unique(keys, axis=0, return_inverse=True)
    depth = points @ direction
    top = np.full(cells.max() + 1, -np.inf)
    np.maximum.at(top, cells.ravel(), depth)
    return np.nonzero(depth >= top[cells.ravel()] - 1.)[0]


class ScanPlan:
    def __init__(self, device):
        self.d, s = device, device.c['scan']
        vehicle = device.vehicle
        self.points, index = sample(vehicle.triangles, s['sample_mm'])
        self.normals, self.parts = normals(vehicle.triangles)[index], vehicle.parts[index]
        self.obstacle = sample(vehicle.triangles, s['obstacle_mm'])[0]
        self.arm = SimpleArm(device.c['robot'], device.c['probe'])
        cells = np.floor(self.points / s['check_mm']).astype(np.int64)
        _, self.check_cell = np.unique(cells, axis=0, return_inverse=True)
        self.check_cell = self.check_cell.ravel()

    def state(self, heading, servo, direction, centre):
        """(visible, reachable) sample indices of one state."""
        s, d = self.d.c['scan'], self.d
        pose = d.vehicle_pose(heading, servo, centre)
        points, outward = apply(pose, self.points), self.normals @ pose[:3, :3].T
        visible = outermost(points, direction, s['cell_mm'])
        visible = visible[outward[visible] @ direction >= np.cos(np.radians(s['max_tilt_deg']))]
        visible = visible[np.isin(self.parts[visible], PARTS)]
        clearance = Clearance(np.concatenate([apply(pose, self.obstacle), d.obstacles(heading, servo, s['obstacle_mm'], centre)]),
                              d.c['probe'], d.c['robot'], s['table_margin_mm'], s['obstacle_mm'])
        good = []
        _, first = np.unique(self.check_cell[visible], return_index=True)
        for i in visible[first]:   # one test per check cell, spread to the cell's visible samples
            links = self.arm.links(points[i], direction)
            if links is not None and clearance.clear(points[i], direction, links):
                good.append(self.check_cell[i])
        return visible, visible[np.isin(self.check_cell[visible], good)]

    def run(self, log=print):
        s = self.d.c['scan']
        seen, reached, rows = np.zeros(len(self.points), bool), np.zeros(len(self.points), bool), []
        for name, spec in s['directions'].items():
            direction = np.asarray(spec['normal'], float)
            for heading in s['headings_deg']:
                for lift, servo in s['lift'].items():
                    visible, ok = self.state(heading, servo, direction, spec['centre_mm'])
                    seen[visible] = reached[ok] = True
                    row = dict(direction=name, heading=heading, lift=lift)
                    for part in PARTS:
                        row[part] = (int(np.sum(self.parts[ok] == part)), int(np.sum(self.parts[visible] == part)))
                    rows.append(row)
                    log(f"{name:5s} heading {heading:3d} lift {lift:4s}: " + ', '.join(
                        f'{p} {a}/{v}' for p, (a, v) in ((p, row[p]) for p in PARTS)))
        coverage = {part: dict(reachable=int(np.sum(reached & (self.parts == part))),
                               visible=int(np.sum(seen & (self.parts == part)))) for part in PARTS}
        return rows, coverage
