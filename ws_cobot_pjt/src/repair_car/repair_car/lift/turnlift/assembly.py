"""hardware/lift_assembly.stl, the team's CAD export of the 90 x 60 lift, as the rigid links and pin axes of a multibody
model. The STL carries no mates, so its connected pieces are grouped by the pin axes in device.yaml `linkage` (STL frame:
x length, y down, z width; mm): thin pieces (pins, clips, spacers, bars, crank hub) go to the link of the nearest axis,
the arms by their diagonal, the crank and the pushrod by the axes they span, the rest above the middle pin to the top
and below it to the base. Left / right twins move together in the plane and are one link.
Model frame: X length, Y width, Z up, origin at the base bottom centre. One degree of freedom, the crank angle: 0 at the
assembled pose (crank along +X, bottom slider farthest out, lift down), + raises."""
import numpy as np

from .mesh import MassProperties, heightmap, read_stl
from .vehicle import connected

AXIS_LINK = dict(crank='crank', crank_pin='pushrod', slider_bottom='slider_bottom', slider_top='slider_top',
                 middle='arm_b', fixed_bottom='base', fixed_top='top')
# kinematic tree: (joint, type, parent, child, axis at the child link origin)
TREE = [('servo', 'revolute', 'base', 'crank', 'crank'),
        ('crank_pin', 'revolute', 'crank', 'pushrod', 'crank_pin'),
        ('rod_bottom', 'revolute', 'pushrod', 'slider_bottom', 'slider_bottom'),
        ('arm_b_bottom', 'revolute', 'slider_bottom', 'arm_b', 'slider_bottom'),
        ('middle', 'revolute', 'arm_b', 'arm_a', 'middle'),
        ('arm_a_top', 'revolute', 'arm_a', 'slider_top', 'slider_top'),
        ('fixed_top', 'revolute', 'arm_b', 'top', 'fixed_top')]
# loop closures: parent -(joint)-> stub link at the axis, the stub welded to the child
CLOSURES = [('fixed_bottom', 'revolute', 'base', 'arm_a', 'fixed_bottom'),
            ('slot_bottom', 'prismatic', 'base', 'slider_bottom', 'slider_bottom'),
            ('slot_top', 'prismatic', 'top', 'slider_top', 'slider_top')]
LIFTED = dict(top=1., slider_top=1., arm_a=.5, arm_b=.5)   # share of each link's weight raised at the top's rate


def segment_distance(p, a, b):
    t = np.clip(np.dot(p - a, b - a) / np.dot(b - a, b - a), 0, 1)
    return np.linalg.norm(p - a - t * (b - a))


class LiftAssembly:
    def __init__(self, spec, lift_spec, root):
        axes = {k: np.asarray(v, float) for k, v in spec['axes'].items()}
        groups = {}
        # `scale` [X length, Y width, Z height] resizes the CAD after its pieces are grouped (STL x length, y down, z width)
        sx, sy, sz = spec.get('scale', (1., 1., 1.))
        for piece in self.pieces(read_stl(root / spec['mesh'])):
            link, pin = self.classify(piece, axes)
            groups.setdefault(link, []).append((piece * [sx, sz, sy], pin))
        axes = {k: v * [sx, sz] for k, v in axes.items()}
        spec = {**spec, 'slot_x': np.asarray(spec['slot_x'], float) * sx}
        base = np.concatenate([p for p, _ in groups['base']]).reshape(-1, 3)
        low, high = base.min(0), base.max(0)
        self.offset = np.array([(low[0] + high[0]) / 2, high[1], (low[2] + high[2]) / 2])   # x, floor y, z
        self.axes = {k: np.array([v[0] - self.offset[0], self.offset[1] - v[1]]) for k, v in axes.items()}   # X, Z
        self.origins = dict(base=np.zeros(3), **{child: self.point(axis) for _, _, _, child, axis in TREE})
        density, shell, material = lift_spec['density_g_cm3'], lift_spec['shell'], spec['material']
        fill = lambda kind, infill: density[kind] * (shell + (1 - shell) * infill)
        self.links = {}
        for link, pieces in groups.items():
            local = [(self.model(p) - self.origins[link], material['pins' if pin else link]) for p, pin in pieces]
            self.links[link] = dict(triangles=np.concatenate([t for t, _ in local]),
                                    **self.combine([MassProperties(t, fill(*m)) for t, m in local]))
        top = self.links['top']['triangles'].reshape(-1, 3)
        self.holder = np.r_[(top[:, :2].min(0) + top[:, :2].max(0)) / 2, top[:, 2].max()]   # holder face centre, top frame
        self.kinematics(spec['slot_x'])

    @staticmethod
    def pieces(triangles):
        _, vertex = np.unique(triangles.reshape(-1, 3).round(4), axis=0, return_inverse=True)
        vertex = vertex.reshape(-1, 3)
        label = connected(np.r_[vertex[:, :2], vertex[:, 1:]], vertex.max() + 1)[vertex[:, 0]]
        return [triangles[label == k] for k in range(label.max() + 1)]

    @staticmethod
    def classify(piece, axes):
        """(link, pin-like) of a connected piece in the STL frame."""
        props, xy = MassProperties(piece), piece.reshape(-1, 3)[:, :2]
        centre = props.centre[:2]
        if (xy.max(0) - xy.min(0)).max() <= 15:
            nearest = min(axes, key=lambda k: np.linalg.norm(axes[k] - centre))
            return AXIS_LINK[nearest], props.volume < 2000
        if np.linalg.norm(centre - axes['middle']) < 2:
            d = xy - centre
            along = lambda a, b: np.mean((d @ (axes[b] - axes[a])) ** 2)
            return ('arm_a' if along('fixed_bottom', 'slider_top') > along('fixed_top', 'slider_bottom') else 'arm_b'), False
        if segment_distance(centre, axes['crank'], axes['crank_pin']) < 2:
            return 'crank', False
        if segment_distance(centre, axes['crank_pin'], axes['slider_bottom']) < 2:
            return 'pushrod', False
        return ('top' if centre[1] < axes['middle'][1] else 'base'), False

    def model(self, triangles):
        """STL frame -> model frame (a rotation: winding kept)."""
        x, y, z = np.moveaxis(triangles, -1, 0)
        return np.stack([x - self.offset[0], z - self.offset[2], self.offset[1] - y], -1)

    def point(self, axis):
        return np.array([self.axes[axis][0], 0., self.axes[axis][1]])

    @staticmethod
    def combine(parts):
        """mass kg, centre mm, inertia kg mm^2 about the centre."""
        mass = sum(p.mass for p in parts)
        centre = sum(p.mass * p.centre for p in parts) / mass
        inertia = sum(p.inertia + p.mass * (np.dot(r, r) * np.eye(3) - np.outer(r, r)) for p in parts
                      for r in [p.centre - centre])
        return dict(mass=mass, centre=centre, inertia=inertia)

    # ---------------- kinematics (model X, Z; mm, rad) ----------------
    def kinematics(self, slot_x):
        c, p, s, f, t = (self.axes[k] for k in ('crank', 'crank_pin', 'slider_bottom', 'fixed_bottom', 'slider_top'))
        self.radius, self.rod, self.arm = np.linalg.norm(p - c), np.linalg.norm(s - p), np.linalg.norm(t - f)
        self.phase = np.arctan2(*(p - c)[::-1])
        self.lock = np.arcsin(min(1., (s[1] + self.rod - c[1]) / self.radius)) - self.phase   # crank tip above the slider
        self.slot = np.asarray(slot_x) - self.offset[0]
        angles = np.linspace(0, self.lock, 4001)
        self.stop = float(np.interp(self.slot[0], self.slider_x(angles)[::-1], angles[::-1]))   # slider at the slot end

    def slider_x(self, angle):
        c, s = self.axes['crank'], self.axes['slider_bottom']
        tip_x = c[0] + self.radius * np.cos(self.phase + angle)
        tip_z = c[1] + self.radius * np.sin(self.phase + angle)
        return tip_x + np.sqrt(np.maximum(self.rod ** 2 - (s[1] - tip_z) ** 2, 0))

    def rise(self, slider_x):
        """Top plate rise (mm) at a bottom slider X."""
        height = lambda x: np.sqrt(self.arm ** 2 - (x - self.axes['fixed_bottom'][0]) ** 2)
        return height(slider_x) - height(self.axes['slider_bottom'][0])

    def rest(self, triangles, cell=1.):
        """How far (mm) to raise a mesh given about the holder top centre so that it rests on the holder (top link)
        without overlap: the largest gap between the holder's upper and the mesh's lower height maps."""
        holder = self.links['top']['triangles'] - self.holder
        low, high = holder.reshape(-1, 3).min(0)[:2], holder.reshape(-1, 3).max(0)[:2]
        shape = tuple(np.ceil((high - low) / cell).astype(int) + 1)
        near = ((triangles[..., :2] >= low - cell) & (triangles[..., :2] <= high + cell)).all((1, 2))
        under = heightmap(triangles[near], low, shape, cell, top=False)
        over = heightmap(holder, low, shape, cell)
        both = np.isfinite(under) & np.isfinite(over)
        return float(np.max(over[both] - under[both]))

    def table(self, down, up, count=141):
        """Firmware lift table (firmware/lift_table.json): servo deg from DOWN to UP (crank = |servo - DOWN|) and the
        holder top above the base bottom (mm)."""
        servo = np.linspace(down, up, count)
        top = self.origins['top'][2] + self.holder[2] + self.rise(self.slider_x(np.radians(np.abs(servo - down))))
        return dict(servo_deg=[round(float(v), 2) for v in servo], top_mm=[round(float(v), 2) for v in top])

    def lifted(self):
        return sum(self.links[k]['mass'] * share for k, share in LIFTED.items())

    def torque(self, mass, g=9.81):
        """Static crank torque (N m) holding `mass` kg at the top over 0 .. slot stop: (max, at deg)."""
        angles = np.linspace(0, self.stop, 2001)
        rate = np.gradient(self.rise(self.slider_x(angles)), angles) / 1000
        i = int(np.argmax(rate))
        return mass * g * rate[i], np.degrees(angles[i])
