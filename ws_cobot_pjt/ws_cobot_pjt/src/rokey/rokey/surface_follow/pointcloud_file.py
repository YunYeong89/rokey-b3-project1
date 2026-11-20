"""ASCII point clouds: metre coordinates and optional named scalar properties."""
from pathlib import Path

import numpy as np


class PointCloudFile:
    @staticmethod
    def save(path, points):
        path = Path(path)
        points = np.asarray(points, dtype=float).reshape(-1, 4)
        if path.suffix == '.plt':
            header, comments = 'x_m y_m z_m normal_force_n', '# '
        elif path.suffix == '.ply':
            return PointCloudFile.save_ply(path, points, properties=('x', 'y', 'z', 'normal_force_n'))
        else:
            raise ValueError('Point-cloud format must be plt or ply')
        np.savetxt(path, points, header=header, comments=comments)
        return path

    @staticmethod
    def save_ply(path, points, frame='base_link', properties=('x', 'y', 'z'), comments=()):
        path = Path(path)
        points = np.asarray(points, dtype=float).reshape(-1, len(properties))
        units = 'coordinates in metres'
        if 'normal_force_n' in properties:
            units += '; normal_force_n in newtons'
        header = '\n'.join([
            'ply', 'format ascii 1.0', f'comment frame {frame}', f'comment {units}',
            *(f'comment {comment}' for comment in comments),
            f'element vertex {len(points)}',
            *(f'property double {name}' for name in properties), 'end_header',
        ])
        np.savetxt(path, points, header=header, comments='')
        return path

    @staticmethod
    def load_ply(path):
        """Vertex rows of an ASCII PLY written by save_ply."""
        lines = Path(path).read_text().splitlines()
        return np.loadtxt(lines[lines.index('end_header') + 1:], ndmin=2)
