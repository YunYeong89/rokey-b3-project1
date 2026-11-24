"""Sample every configured vehicle mesh into PLY, without ROS or simulation.

python -m repair_car.pointcloud.mesh_to_pointcloud --spacing-mm 1
Coordinates: vehicle frame (x front, y left, z up), footprint-centred, metres.
"""
import argparse
from math import isfinite
from pathlib import Path

from repair_car.config import PACKAGE, load
from repair_car.pointcloud.pointcloud_file import PointCloudFile
from repair_car.geometry.vehicle_model import VehicleModel


class VehiclePointCloudExporter:
    def __init__(self, output, spacing_mm=1.0):
        self.output, self.spacing = Path(output), spacing_mm

    def export(self, kind):
        model = VehicleModel(kind)
        points, _ = model.sample(self.spacing)
        self.output.mkdir(parents=True, exist_ok=True)
        path = PointCloudFile.save_ply(self.output / f'{kind}_mesh.ply', points / 1000,
            frame=f'vehicle_{kind}', comments=(
                f'source {model.spec["mesh"]}',
                f'mm_per_source_unit {model.spec["mm_per_unit"]}',
                'axes x front, y left, z up; origin footprint centre on ground',
                f'area sampling density approximately one point per {self.spacing ** 2:g} mm^2',
            ))
        return path, len(points)


def main():
    kinds = tuple(load('vehicles')['types'])
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--vehicles', nargs='+', choices=kinds, default=kinds)
    parser.add_argument('--spacing-mm', type=float, default=1.0)
    parser.add_argument('--output', type=Path, default=PACKAGE / 'pointcloud')
    args = parser.parse_args()
    if not isfinite(args.spacing_mm) or args.spacing_mm <= 0:
        parser.error('--spacing-mm must be finite and positive')
    exporter = VehiclePointCloudExporter(args.output, args.spacing_mm)
    for kind in args.vehicles:
        path, count = exporter.export(kind)
        print(f'{kind}: {count:,} points -> {path}')


if __name__ == '__main__':
    main()
