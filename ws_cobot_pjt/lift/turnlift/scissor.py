"""The MG996R scissor lift of hardware/source/compact/lc.py (imported: the printed parts' geometry is the only source).
Lift frame: x along the length from the front wall (0..120), y across (centre 0), z from the base bottom."""
import sys

import numpy as np

from .mesh import MassProperties, read_stl


class ScissorLift:
    def __init__(self, spec, root):
        for sub in ('compact', 'lift3d'):
            path = str(root / 'hardware/source' / sub)
            if path not in sys.path:
                sys.path.insert(0, path)
        import lc
        self.lc, self.spec = lc, spec
        self.down, self.up = spec['servo_down_up_deg']
        self.size = np.array([lc.BL, lc.BW])
        self.parts = self.masses(root / 'hardware/stl')
        self.mass = sum(p['mass'] for p in self.parts) + spec['servo_mass_kg']
        self.rising = sum(p['mass'] * p['rise'] for p in self.parts)  # effective mass lifted at the top's rate

    def crank(self, servo):
        """Crank angle (deg) at a servo angle: servo down -> TH_DOWN, servo up -> TH_DOWN + TH_SPAN."""
        lc = self.lc
        return lc.TH_DOWN + (self.down - servo) / (self.down - self.up) * lc.TH_SPAN

    def top(self, servo):
        """Top plate upper surface above the base bottom (mm)."""
        lc = self.lc
        return lc.ZP + lc.h_of(self.crank(servo)) + lc.TOP_ROD_BELOW + lc.TOP_T

    def ratio(self, servo, step=.25):
        """|d top / d servo angle| in mm per rad: the lift's transmission at that angle."""
        return abs(self.top(servo + step) - self.top(servo - step)) / np.radians(2 * step)

    def angles(self, count=141):
        return np.linspace(self.down, self.up, count)

    def servo_for(self, top):
        """Servo angle for a top height (the stroke is monotone)."""
        angles = self.angles(561)
        return float(np.interp(top, [self.top(a) for a in angles], angles))

    def masses(self, directory):
        """Printed parts from their STL volumes and the parts list's material / infill; `rise` is how much of the
        part's weight is lifted at the top plate's rate."""
        density, shell, rows = self.spec['density_g_cm3'], self.spec['shell'], []
        files = sorted(directory.glob('*.stl'))
        for path in files:
            key = path.name[:2]
            material, infill, rise, *used = self.spec['parts'][key]
            count = used[0] / sum(f.name[:2] == key for f in files) if used else int(path.stem.rsplit('_x', 1)[1])
            fill = shell + (1 - shell) * infill
            mass = MassProperties(read_stl(path), density[material] * fill).mass * count
            rows.append(dict(part=path.stem, material=material, infill=infill, count=count, mass=mass, rise=rise))
        return rows
