"""Prepare shallow width-series cracks and the largest repository probe, keeping the originals."""
import argparse
import hashlib
import json

import numpy as np

from .configs import PACKAGE
from .real_vehicle import read_stl
from .vehicle_model import write_stl


class ProbeMesh:
    def __init__(self):
        self.directory = PACKAGE / 'meshes'
        self.source = self.directory / 'probes/source.json'
        self.metadata = json.loads(self.source.read_text())

    @staticmethod
    def sphere(triangles):
        vertices = np.unique(triangles.reshape(-1, 3), axis=0)
        cap = vertices[vertices[:, 1] <= vertices[:, 1].min() + 1.]
        fit = np.linalg.lstsq(np.c_[2 * cap, np.ones(len(cap))], (cap * cap).sum(1), rcond=None)[0]
        radius = float(np.sqrt(fit[3] + fit[:3] @ fit[:3]))
        return fit[:3], radius

    def prepare(self):
        variants = []
        for item in self.metadata['original_meshes']:
            path = self.directory / item['file']
            centre, radius = self.sphere(read_stl(path))
            variants.append((path, centre, radius))
        path, centre, radius = max(variants, key=lambda item: (round(item[2], 3), item[0].stem.startswith('probe_v2')))
        transform = np.asarray(self.metadata['source_to_tip_frame'])
        triangles = read_stl(path) @ transform[:3, :3].T + transform[:3, 3]
        output = self.directory / path.name
        write_stl(output, triangles)
        centre = np.round(transform[:3, :3] @ centre + transform[:3, 3], 6).tolist()
        radius = round(radius, 3)
        self.metadata.update(active_mesh=output.name, active_sha256=hashlib.sha256(output.read_bytes()).hexdigest(),
                             tip_sphere=dict(centre_mm=centre, radius_mm=radius))
        self.source.write_text(json.dumps(self.metadata, ensure_ascii=False, indent=2) + '\n')
        return dict(mesh=f'meshes/{output.name}', radius_mm=radius, diameter_mm=2 * radius,
                    centre_mm=centre, variants=[dict(mesh=p.name, diameter_mm=round(2 * r, 3))
                                               for p, _, r in variants])


class CouponMesh:
    def __init__(self, depth_mm):
        self.depth = depth_mm
        self.source = PACKAGE / 'meshes/fracture_sample.stl'
        self.output = PACKAGE / 'meshes/fracture_sample_shallow.stl'

    def prepare(self):
        triangles = read_stl(self.source).copy()
        # Source is Y-up. Only the five width-series slots at specimen Y=69..85 mm change.
        x, h, z = triangles[:, :, 0], triangles[:, :, 1], triangles[:, :, 2]
        selected = (x >= -95.) & (x <= -40.) & (z >= -90.) & (z <= -65.) & (h > 0.) & (h < 10.)
        old_depth = float((10. - h[selected]).max())
        h[selected] = 10. - (10. - h[selected]) * self.depth / old_depth
        write_stl(self.output, triangles)
        result = dict(source=self.source.name, source_sha256=hashlib.sha256(self.source.read_bytes()).hexdigest(),
                      mesh=self.output.name, sha256=hashlib.sha256(self.output.read_bytes()).hexdigest(),
                      units='mm', source_up='+Y', specimen_size_mm=[200., 200., 10.],
                      width_series=dict(widths_mm=[2.5, 2., 1.5, 1., .5], old_depth_mm=old_depth,
                                        depth_mm=self.depth, local_bounds_mm=[-95., 65., -40., 90.]),
                      changed_vertex_occurrences=int(selected.sum()))
        self.output.with_suffix('.json').write_text(json.dumps(result, indent=2) + '\n')
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--crack-depth-mm', type=float, default=.5)
    options = parser.parse_args()
    if not 0 < options.crack_depth_mm < 5:
        parser.error('--crack-depth-mm must be between 0 and the original 5 mm depth')
    print(json.dumps(dict(probe=ProbeMesh().prepare(), coupon=CouponMesh(options.crack_depth_mm).prepare()), indent=2))


if __name__ == '__main__':
    main()
