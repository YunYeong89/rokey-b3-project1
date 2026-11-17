"""Compare part scans without ROS or robot motion.

Examples (project PLY coordinates are metres; ROI bounds and thresholds are millimetres)::

    python -m rokey.pointcloud_damage --part bonnet --previous before.ply --measured after.ply
    python -m rokey.pointcloud_damage --manifest parts.json --output comparison

Use --scene-pose scene_truth.json for vehicle-frame references versus base_link measurements.
Supply the pose captured with the scan; a current session pose need not match a historical scan.

Manifest: {"parts": [{"part": "bonnet", "previous": "before.ply", "measured": "after.ply",
                     "roi": "roi_plan.json", "axis": [0, 0, 1],
                     "settings": {"tolerance_mm": 0.5, "full_damage_mm": 5}}]}.
ROI: {"frame": "base_link", "regions": [{"id": "roi_1", "bounds_mm": [x0,y0,x1,y1]}]}.
Six bounds mean [x0,y0,z0,x1,y1,z1]. Existing block_local ROI plans are translated by
centre_base_mm. role="anchor" supplies known healthy alignment anchors. coverage_complete
allows missing candidates; missing_confirmed (e.g. an independently verified no-contact
region) confirms missing material. Absent points alone remain unobserved by default.

By default all 3-D surface voxels are compared. An explicit --axis selects the outer surface
visible from that direction, useful for one-sided probe scans. Crack labels are geometric candidates.
Percentages are geometric indices, not fracture probabilities or remaining strength.
"""
import argparse
import csv
import json
from dataclasses import asdict, dataclass
from datetime import datetime
from itertools import product
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation


@dataclass
class Settings:
    cell_mm: float = 1.
    tolerance_mm: float = .5
    full_damage_mm: float = 5.
    match_radius_mm: float = 1.5
    max_deviation_mm: float = 20.
    registration_radius_mm: float = 10.
    registration_iterations: int = 30
    min_region_cells: int = 3
    crack_width_mm: float = 3.
    crack_aspect: float = 4.
    normal_distance_weight: float = .05
    max_normal_angle_deg: float = 45.
    align: bool = True

    def __post_init__(self):
        lengths = ['cell_mm','tolerance_mm','full_damage_mm','match_radius_mm',
                   'max_deviation_mm','registration_radius_mm','crack_width_mm','crack_aspect',
                   'normal_distance_weight','max_normal_angle_deg']
        if (not all(np.isfinite(getattr(self,k)) and getattr(self,k)>0 for k in lengths)
                or self.full_damage_mm <= self.tolerance_mm
                or self.max_normal_angle_deg >= 90
                or self.registration_iterations < 1 or self.min_region_cells < 3):
            raise ValueError('Positive lengths, full_damage_mm > tolerance_mm, iterations >= 1 and min_region_cells >= 3 are required')


class PointCloud:
    """Project ASCII PLY/PLT/XYZ/CSV/NPY and scalar-vertex binary PLY; extra fields are ignored."""
    PLY_TYPES = {'char': 'i1', 'uchar': 'u1', 'int8': 'i1', 'uint8': 'u1',
                 'short': 'i2', 'ushort': 'u2', 'int16': 'i2', 'uint16': 'u2',
                 'int': 'i4', 'uint': 'u4', 'int32': 'i4', 'uint32': 'u4',
                 'float': 'f4', 'double': 'f8', 'float32': 'f4', 'float64': 'f8'}

    def __init__(self, points, frame='', path=None):
        points = np.asarray(points, float)
        if not points.size:
            points = np.empty((0,3))
        if points.ndim != 2 or points.shape[1] < 3:
            raise ValueError('A point cloud needs at least three XYZ columns')
        self.points = points[np.isfinite(points[:, :3]).all(axis=1), :3]
        self.frame, self.path = frame, str(path) if path else None

    @classmethod
    def load(cls, path, units='m'):
        path, frame = Path(path), ''
        if path.suffix.lower() == '.ply':
            with path.open('rb') as stream:
                if stream.readline().strip() != b'ply':
                    raise ValueError(f'Invalid PLY header: {path}')
                properties, element, count, encoding = [], '', 0, ''
                while True:
                    raw = stream.readline()
                    if not raw:
                        raise ValueError(f'Unterminated PLY header: {path}')
                    line = raw.decode('ascii').strip().split()
                    if line[:1] == ['format']:
                        encoding = line[1]
                    elif line[:2] == ['comment', 'frame']:
                        frame = ' '.join(line[2:])
                    elif line[:1] == ['element']:
                        element = line[1]
                        if element == 'vertex':
                            count = int(line[2])
                    elif line[:1] == ['property'] and element == 'vertex':
                        if line[1] == 'list':
                            raise ValueError('List-valued vertex properties are unsupported')
                        properties.append((line[2], cls.PLY_TYPES[line[1]]))
                    elif line == ['end_header']:
                        break
                names = [name for name, _ in properties]
                if not count:
                    points = np.empty((0,3))
                elif encoding == 'ascii':
                    points = np.loadtxt(stream, max_rows=count, usecols=[names.index(k) for k in 'xyz'], ndmin=2)
                elif encoding in ('binary_little_endian', 'binary_big_endian'):
                    endian = '<' if encoding == 'binary_little_endian' else '>'
                    records = np.fromfile(stream, dtype=[(n, endian + t) for n, t in properties], count=count)
                    points = np.column_stack([records[k] for k in 'xyz'])
                else:
                    raise ValueError(f'Unsupported PLY encoding: {encoding}')
                if len(points) != count:
                    raise ValueError(f'PLY vertex count does not match its data: {path}')
        elif path.suffix.lower() == '.npy':
            points = np.load(path, allow_pickle=False)
        elif path.suffix.lower() == '.csv':
            with path.open() as stream:
                first = stream.readline().strip().lstrip('#').split(',')
            header = any(k.lower() in ('x', 'x_m', 'x_mm') for k in first)
            if header:
                columns = [[i for i, k in enumerate(first) if k.lower() in (axis, axis+'_m', axis+'_mm')][0]
                           for axis in 'xyz']
                if all(first[i].lower().endswith('_mm') for i in columns):
                    units = 'mm'
                elif all(first[i].lower().endswith('_m') for i in columns):
                    units = 'm'
            else:
                columns = (0, 1, 2)
            points = np.loadtxt(path, delimiter=',', skiprows=int(header), usecols=columns, ndmin=2)
        elif path.suffix.lower() in ('.plt', '.xyz', '.txt'):
            points = np.loadtxt(path, ndmin=2)
        else:
            raise ValueError('Supported point clouds: PLY, PLT, XYZ, TXT, CSV, NPY')
        return cls(points[:, :3] * (.001 if units == 'mm' else 1.), frame, path)

    @staticmethod
    def voxel(points, size):
        if not len(points):
            return points.copy()
        _, inverse = np.unique(np.floor(np.round(points / size, 9)).astype(np.int64), axis=0, return_inverse=True)
        count = np.bincount(inverse)
        return np.column_stack([np.bincount(inverse, weights=points[:, i]) / count for i in range(3)])


class ROI:
    def __init__(self, data, frame, to_reference=None):
        self.data = dict(data)
        self.id = str(data.get('id', 'roi'))
        self.role = data.get('role', 'analysis')
        self.complete = bool(data.get('coverage_complete', False))
        self.missing = bool(data.get('missing_confirmed', False))
        bounds = np.asarray(data['bounds_mm'], float) / 1000
        if bounds.shape == (4,):
            self.low, self.high = np.r_[bounds[:2], -np.inf], np.r_[bounds[2:], np.inf]
        elif bounds.shape == (6,):
            self.low, self.high = bounds[:3], bounds[3:]
        else:
            raise ValueError('ROI bounds_mm must have four XY or six XYZ bounds')
        if not np.isfinite(bounds).all() or np.any(self.low >= self.high):
            raise ValueError(f'Invalid ROI bounds: {self.id}')
        self.frame = frame
        self.inverse = np.linalg.inv(to_reference) if to_reference is not None else None
        if to_reference is not None:
            self.data.update(reference_frame=frame, roi_to_reference=to_reference.tolist())

    def mask(self, points):
        if self.inverse is not None:
            points = points @ self.inverse[:3,:3].T + self.inverse[:3,3]
        return ((points >= self.low) & (points <= self.high)).all(axis=1)

    @classmethod
    def load(cls, source, part, frame, measured_to_reference=None, measured_frame=''):
        if source is None:
            return []
        data = json.loads(Path(source).read_text()) if isinstance(source, (str, Path)) else source
        if isinstance(data, list):
            data = {'frame': frame, 'regions': data}
        roi_frame = data.get('frame', frame)
        result = []
        for index, original in enumerate(data.get('regions', [])):
            if original.get('part', part) != part:
                continue
            region = {'id': f'roi_{index+1:03d}', **original}
            current_frame = region.get('frame', roi_frame)
            if current_frame == 'block_local':
                centre = np.asarray(data['centre_base_mm'], float)
                bounds = np.asarray(region['bounds_mm'], float)
                if len(bounds) != 4 or len(centre) < 2:
                    raise ValueError('Existing block_local plans require four XY bounds and centre_base_mm')
                region.update(source_frame=current_frame, source_bounds_mm=bounds.tolist(),
                              source_to_base_xy_mm=centre[:2].tolist())
                region['bounds_mm'] = (bounds + np.tile(centre[:2], 2)).tolist()
                current_frame = 'base_link'
            to_reference = None
            if current_frame != frame:
                if current_frame != measured_frame or measured_to_reference is None:
                    raise ValueError(f'ROI frame {current_frame!r} differs from reference frame {frame!r}; supply its measured-to-reference transform')
                to_reference = measured_to_reference
            region['frame'] = current_frame
            result.append(cls(region, frame, to_reference))
        if not result:
            raise ValueError(f'No ROI regions for part {part!r}')
        if len({r.id for r in result}) != len(result):
            raise ValueError('ROI identifiers must be unique within a part')
        return result


class Surface:
    """Density-normalised 3-D surface, or an explicitly requested one-sided projected view."""
    def __init__(self, points, axis, cell):
        self.full_3d = axis is None
        values, vectors = np.linalg.eigh(np.cov(points.T))
        inferred = vectors[:, 0]
        inferred *= np.sign(inferred[np.argmax(np.abs(inferred))])
        self.axis = np.asarray(inferred if axis is None else axis, float)
        if self.axis.shape != (3,) or not np.isfinite(self.axis).all() or np.linalg.norm(self.axis) < 1e-9:
            raise ValueError('The outward viewing axis must be a nonzero finite XYZ vector')
        self.axis /= np.linalg.norm(self.axis)
        seed = np.eye(3)[np.argmin(np.abs(self.axis))]
        u = np.cross(self.axis, seed); u /= np.linalg.norm(u)
        self.basis = np.column_stack([u, np.cross(self.axis, u)])
        if self.full_3d:
            self.points = PointCloud.voxel(points, cell)
            self.keys = np.floor(np.round(self.points/cell, 9)).astype(np.int64)
        else:
            uv = points @ self.basis
            keys = np.floor(np.round(uv / cell, 9)).astype(np.int64)
            order = np.argsort(-(points @ self.axis), kind='stable')
            _, first = np.unique(keys[order], axis=0, return_index=True)
            self.points, self.keys = points[order[first]], keys[order[first]]
        if len(self.points) < 6:
            raise ValueError('Too few projected cells; change the viewing axis or cell size')
        self.uv, self.cell = self.points @ self.basis, cell
        self.normals, self.reliable = self.estimate_normals(self.points)
        if self.full_3d and values[0] > values[1]*.05:
            orientation = np.einsum('ij,ij->i', self.normals, self.points-self.points.mean(axis=0))
        else:
            orientation = self.normals @ self.axis
        self.normals[orientation < 0] *= -1
        if self.full_3d:
            self.area = np.full(len(self.points), cell**2)
        else:
            self.reliable &= self.normals @ self.axis > .2
            self.area = cell**2 / np.maximum(self.normals @ self.axis, .2)

    @staticmethod
    def estimate_normals(points):
        if len(points) < 6:
            return np.zeros_like(points), np.zeros(len(points),bool)
        neighbours = cKDTree(points).query(points,k=min(12,len(points)))[1]
        local = points[neighbours]
        centred = local-local.mean(axis=1,keepdims=True)
        values, vectors = np.linalg.eigh(np.einsum('nki,nkj->nij',centred,centred))
        reliable = (values[:,1]>values[:,2]*.02) & (values[:,0]<values[:,1]*.3)
        return vectors[:,:,0], reliable

    def match(self, measured, settings):
        if not self.full_3d:
            lateral, nearest = cKDTree(measured @ self.basis).query(self.uv)
            supported = lateral <= settings.match_radius_mm/1000
            return nearest, supported, supported
        # Search along each local normal, rather than matching a deep dent to its healthy rim.
        radius, depth = settings.match_radius_mm/1000, settings.max_deviation_mm/1000
        offsets = np.linspace(-depth, depth, int(np.ceil(2*depth/radius))+1)
        tree = cKDTree(measured)
        measured_normals, measured_reliable = self.estimate_normals(measured)
        cosine = np.cos(np.deg2rad(settings.max_normal_angle_deg))
        nearest, supported = np.zeros(len(self.points), int), np.zeros(len(self.points), bool)
        spatial_support = supported.copy()
        for first in range(0, len(self.points), 512):
            points, normals = self.points[first:first+512], self.normals[first:first+512]
            probes = points[:, None]+normals[:, None]*offsets[None, :, None]
            candidate = tree.query(probes.reshape(-1,3))[1].reshape(len(points),-1)
            delta = measured[candidate]-points[:, None]
            along = np.einsum('nki,ni->nk', delta, normals)
            across_squared = np.maximum(0., np.sum(delta**2,axis=2)-along**2)
            valid = (across_squared <= radius**2) & (np.abs(along) <= depth)
            spatial_support[first:first+len(points)] = valid.any(axis=1)
            compatible = np.abs(np.einsum('nki,ni->nk',measured_normals[candidate],normals)) >= cosine
            # Do not mistake a different-facing sheet for deformation of an unobserved side.
            # Line-only samples without reliable normals support nearby matches, not remote ray hits.
            valid &= ((measured_reliable[candidate] & compatible)
                      | (~measured_reliable[candidate] & (np.sum(delta**2,axis=2) <= radius**2)))
            cost = np.where(valid, across_squared+(along*settings.normal_distance_weight)**2, np.inf)
            choice = np.argmin(cost,axis=1)
            nearest[first:first+len(points)] = candidate[np.arange(len(points)),choice]
            supported[first:first+len(points)] = valid.any(axis=1)
        return nearest, supported, spatial_support


class RigidRegistration:
    """Trimmed local point-to-plane ICP; no scaling, no per-defect alignment."""
    @staticmethod
    def apply(points, matrix):
        return points @ matrix[:3, :3].T + matrix[:3, 3]

    @staticmethod
    def transform(source, measured_frame='', reference_frame=''):
        if source is None:
            return np.eye(4)
        value = json.loads(Path(source).read_text()) if isinstance(source, (str, Path)) else source
        if isinstance(value, dict):
            if 'pose_base_mm_deg' in value:
                x, y, yaw = value['pose_base_mm_deg']
                matrix = np.eye(4); matrix[:3,:3] = Rotation.from_euler('z',yaw,degrees=True).as_matrix()
                matrix[:3,3] = np.array([x,y,value['base_z_mm']])/1000
                if (measured_frame, reference_frame) == ('base_link','vehicle'):
                    value = np.linalg.inv(matrix)
                elif (measured_frame, reference_frame) == ('vehicle','base_link'):
                    value = matrix
                else:
                    raise ValueError('A scene pose maps only vehicle <-> base_link; supply a 4x4 matrix for other frames')
            else:
                value = value['matrix']
        matrix = np.asarray(value, float)
        if (matrix.shape != (4, 4) or not np.isfinite(matrix).all()
                or not np.allclose(matrix[3], [0, 0, 0, 1])
                or not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-5)
                or not np.isclose(np.linalg.det(matrix[:3, :3]), 1, atol=1e-5)):
            raise ValueError('Initial transform must be rigid; translations are metres')
        return matrix

    def run(self, measured, surface, rois, settings, initial):
        matrix = initial.copy()
        warnings = []
        if not settings.align:
            return matrix, {'status': 'provided_transform', 'matrix': matrix.tolist(), 'warnings': []}
        if not len(measured):
            return matrix, {'status': 'empty_measurement', 'matrix': matrix.tolist(),
                            'warnings': ['An empty scan is not evidence of missing material without independent ROI confirmation.']}
        selected = surface.reliable.copy()
        initial_points = self.apply(measured, matrix)
        source_mask = np.ones(len(measured), bool)
        anchors = [r for r in rois if r.role == 'anchor']
        analysis = [r for r in rois if r.role != 'anchor']
        if anchors:
            selected &= np.logical_or.reduce([r.mask(surface.points) for r in anchors])
            source_mask = np.logical_or.reduce([r.mask(initial_points) for r in anchors])
        elif analysis:
            outside = ~np.logical_or.reduce([r.mask(surface.points) for r in analysis])
            source_outside = ~np.logical_or.reduce([r.mask(initial_points) for r in analysis])
            if np.count_nonzero(selected & outside) >= 12 and np.count_nonzero(source_outside) >= 12:
                selected &= outside; source_mask = source_outside
            else:
                warnings.append('No healthy anchors outside the inspection ROIs; robust alignment uses the whole view.')
        if np.count_nonzero(selected) < 6 or np.count_nonzero(source_mask) < 6:
            raise ValueError('Insufficient healthy alignment samples; supply anchors or use --no-align')
        target, normals = surface.points[selected], surface.normals[selected]
        source = measured[source_mask]
        source = source[np.linspace(0, len(source)-1, min(len(source), 6000), dtype=int)]
        tree, centre = cKDTree(target), target.mean(axis=0)
        scales = np.r_[[.1]*3, [settings.registration_radius_mm/1000]*3]
        rank, rmse, count = 0, None, 0
        for iteration in range(settings.registration_iterations):
            moved = self.apply(source, matrix)
            distance, nearest = tree.query(moved)
            keep = distance <= settings.registration_radius_mm / 1000
            if np.count_nonzero(keep) < 6:
                break
            residual = np.einsum('ij,ij->i', moved-target[nearest], normals[nearest])
            keep &= np.abs(residual) <= max(np.quantile(np.abs(residual[keep]), .7), 1e-8)
            if np.count_nonzero(keep) < 6:
                break
            p, n, error = moved[keep], normals[nearest[keep]], residual[keep]
            design = np.c_[np.cross(p-centre, n), n] * scales
            rank = int(np.linalg.matrix_rank(design, tol=max(np.linalg.norm(design, 2)*1e-6, 1e-12)))
            weight = np.minimum(1., settings.tolerance_mm/1000 / np.maximum(np.abs(error), 1e-12))
            lhs = design * np.sqrt(weight[:, None]); rhs = -error * np.sqrt(weight)
            step = np.linalg.lstsq(np.vstack([lhs, np.eye(6)*.001]), np.r_[rhs, np.zeros(6)], rcond=None)[0] * scales
            rotate = Rotation.from_rotvec(step[:3]).as_matrix()
            delta = np.eye(4); delta[:3, :3] = rotate
            delta[:3, 3] = centre + step[3:] - rotate @ centre
            proposal = delta @ matrix
            relative = proposal @ np.linalg.inv(initial)
            centre_shift = self.apply(centre[None], relative)[0]-centre
            if (np.linalg.norm(centre_shift) > settings.registration_radius_mm / 1000
                    or np.linalg.norm(Rotation.from_matrix(relative[:3, :3]).as_rotvec()) > np.deg2rad(10)):
                warnings.append('Refinement exceeded its local translation/rotation bound; supply a better initial transform.')
                break
            matrix = proposal; count = len(error); rmse = float(np.sqrt(np.mean(error**2))*1000)
            if np.linalg.norm(step[:3]) < 1e-6 and np.linalg.norm(step[3:]) < 1e-7:
                break
        if rank < 6:
            warnings.append('Rigid alignment is not fully observable in this view; unconstrained motion relies on the input frame/initial transform.')
        return matrix, {'status': 'aligned' if count >= 6 else 'insufficient_overlap',
                        'observable_rank': rank, 'inlier_count': count, 'inlier_rmse_mm': rmse,
                        'matrix': matrix.tolist(), 'warnings': warnings}


class PartComparison:
    LABELS = {'unobserved': 0, 'ok': 1, 'dent': 2, 'deformation': 3,
              'crack_candidate': 4, 'missing_candidate': 5, 'missing': 6, 'suspect': 7,
              'deformation_candidate': 8}
    COLORS = np.array([[130,130,130], [45,190,75], [255,145,35], [220,45,40],
                       [165,40,210], [245,210,45], [30,30,30], [255,210,100], [230,180,40]], dtype=int)

    def __init__(self, part, previous, measured, settings, axis=None, rois=(), initial=None, complete=False):
        self.part, self.settings = part, settings
        self.previous, self.measured, self.rois = previous, measured, list(rois)
        self.frame = previous.frame or 'base_link'
        if len(previous.points) < 6:
            raise ValueError('The previous/reference cloud needs at least six finite points')
        provided_transform = initial is not None
        initial = RigidRegistration.transform(initial, measured.frame, self.frame)
        if measured.frame and measured.frame != self.frame and not provided_transform:
            raise ValueError('Different point-cloud frames require an explicit initial transform')
        cell = settings.cell_mm / 1000
        reference = PointCloud.voxel(previous.points, cell/2)
        self.surface = Surface(reference, axis, cell)
        samples = PointCloud.voxel(measured.points, cell/2)
        matrix, self.registration = RigidRegistration().run(samples, self.surface, self.rois, settings, initial)
        self.aligned = RigidRegistration.apply(samples, matrix)
        s = self.surface
        nearest, supported = np.zeros(len(s.points),int), np.zeros(len(s.points),bool)
        spatial_support = supported.copy()
        self.deviation = np.full(len(s.points), np.nan)
        if len(self.aligned):
            nearest, supported, spatial_support = s.match(self.aligned, settings)
            delta = self.aligned[nearest] - s.points
            self.deviation = np.einsum('ij,ij->i', delta, s.normals) * 1000
        self.observed = (supported & s.reliable
                         & (np.abs(self.deviation) <= settings.max_deviation_mm))
        self.complete = np.full(len(s.points), complete, bool)
        self.confirmed_missing = np.zeros(len(s.points), bool)
        self.analysis = np.ones(len(s.points), bool)
        views = [r for r in self.rois if r.role != 'anchor']
        if views:
            self.analysis = np.logical_or.reduce([r.mask(s.points) for r in views])
        for roi in views:
            mask = roi.mask(s.points)
            self.complete[mask] |= roi.complete
            self.confirmed_missing[mask] |= roi.missing
        self.confirmed_missing &= (~self.observed | (self.deviation < -settings.full_damage_mm))
        self.candidate_missing = ~spatial_support & self.complete & ~self.confirmed_missing
        self.candidate_deformation = (spatial_support & ~self.observed & self.complete
                                      & ~self.confirmed_missing)
        self.known = self.observed | self.confirmed_missing
        self.severity = np.clip((np.abs(self.deviation)-settings.tolerance_mm)
                                / (settings.full_damage_mm-settings.tolerance_mm), 0., 1.)
        self.severity[~self.observed] = 0.
        self.severity[self.confirmed_missing] = 1.
        self.labels = np.full(len(s.points), self.LABELS['unobserved'], int)
        self.labels[self.observed] = self.LABELS['ok']
        self.labels[self.confirmed_missing] = self.LABELS['missing']
        self.labels[self.candidate_missing] = self.LABELS['missing_candidate']
        self.labels[self.candidate_deformation] = self.LABELS['deformation_candidate']
        self.changed = (self.observed & (np.abs(self.deviation)>settings.tolerance_mm)) | self.confirmed_missing
        self.labels[self.observed & self.changed & ~self.confirmed_missing] = self.LABELS['suspect']
        self.regions = self.find_regions(nearest)

    def find_regions(self, nearest):
        s, settings = self.surface, self.settings
        active = self.analysis & (self.changed | self.candidate_missing | self.candidate_deformation)
        remaining = {tuple(key): i for i, key in enumerate(s.keys) if active[i]}
        category = np.where(self.confirmed_missing,2,np.where(self.candidate_missing,1,
                            np.where(self.candidate_deformation,3,0)))
        neighbours = list(product((-1,0,1), repeat=s.keys.shape[1]))
        regions = []
        while remaining:
            _, seed = remaining.popitem(); group, pending = [seed], [seed]
            while pending:
                current = pending.pop()
                for offset in neighbours:
                    key = tuple(s.keys[current]+offset); other = remaining.get(key)
                    if other is not None and category[other] == category[seed]:
                        remaining.pop(key); group.append(other); pending.append(other)
            indices = np.asarray(group)
            if len(indices) < settings.min_region_cells:
                # Isolated changes are retained in the scalar cloud, but not promoted to a defect region.
                continue
            points = s.points[indices]
            _, vectors = np.linalg.eigh(np.cov(points.T))
            extent = np.ptp(points @ vectors[:, -2:], axis=0)*1000 + settings.cell_mm
            width, length = float(min(extent)), float(max(extent))
            if category[seed]:
                label = {1:'missing_candidate',2:'missing',3:'deformation_candidate'}[category[seed]]
            else:
                inward = np.mean(self.deviation[indices] < 0)
                label = ('crack_candidate' if inward >= .8 and width <= settings.crack_width_mm
                         and length/width >= settings.crack_aspect else 'dent' if inward >= .8 else 'deformation')
                self.labels[indices] = self.LABELS[label]
            observed = indices[self.observed[indices]]
            bounds_points = np.vstack([points, self.aligned[nearest[observed]]]) if len(observed) else points
            normal = np.mean(s.normals[indices],axis=0)
            normal = normal/np.linalg.norm(normal) if np.linalg.norm(normal)>1e-6 else None
            region = {'id': f'damage_{len(regions)+1:03d}', 'type': label,
                      'confidence': 'confirmed_missing' if label == 'missing' else 'geometric_candidate',
                      'source_roi_ids': [r.id for r in self.rois if r.mask(points).any()],
                      'roi': {'frame': self.frame, 'bounds_mm': (np.r_[bounds_points.min(0), bounds_points.max(0)]*1000).tolist(),
                              'centre_mm': (points.mean(0)*1000).tolist(),
                              'outward_normal': normal.tolist() if normal is not None else None,
                              'view_axis': None if s.full_3d else s.axis.tolist()},
                      'area_mm2': float(s.area[indices].sum()*1e6), 'length_mm': length, 'width_mm': width,
                      'max_deviation_mm': float(np.max(np.abs(self.deviation[observed]))) if len(observed) else None,
                      'severity_percent': (float(100*np.quantile(self.severity[indices], .95))
                                           if len(observed) or label == 'missing' else None),
                      **self.summarize(np.isin(np.arange(len(s.points)), indices))}
            regions.append(region)
        return regions

    def summarize(self, mask):
        weights = self.surface.area
        total = float(weights[mask].sum())
        known = mask & self.known
        assessed = float(weights[known].sum())
        damage = float((weights[known]*self.severity[known]).sum())
        healthy = known & ~self.changed
        unknown = max(0., total-assessed)
        return {'coverage_percent': 100*assessed/total if total else 0.,
                'sync_percent': 100*float(weights[healthy].sum())/assessed if assessed else None,
                'sync_percent_scope': 'assessed surface within geometric tolerance',
                'damage_percent': 100*damage/assessed if assessed else None,
                'damage_percent_scope': 'assessed surface only',
                'whole_roi_damage_bounds_percent': [100*damage/total, 100*(damage+unknown)/total] if total else [None, None],
                'affected_area_percent': 100*float(weights[mask & self.changed].sum())/assessed if assessed else None,
                'unknown_area_mm2': unknown*1e6,
                'missing_candidate_area_mm2': float(weights[mask & self.candidate_missing].sum()*1e6),
                'deformation_candidate_area_mm2': float(weights[mask & self.candidate_deformation].sum()*1e6),
                'reference_cells': int(mask.sum()), 'assessed_cells': int(known.sum()),
                'status': 'no_reference' if not total else 'insufficient_coverage' if not assessed else
                          'partial_coverage' if unknown > total*.05 else 'assessed'}

    def report(self):
        s = self.surface
        warnings = list(self.registration['warnings'])
        if (~self.known & self.analysis).any():
            warnings.append('Unobserved surface is not proven missing; the whole-ROI damage interval includes this uncertainty.')
        if self.candidate_missing.any():
            warnings.append('Complete scan coverage suggests missing candidates; absence alone does not confirm material loss.')
        warnings.append('Crack/dent/deformation labels describe geometry; fracture identity and remaining strength need independent evidence.')
        warnings.append('Analysis cell size is not measurement accuracy; cracks narrower than sampling gaps or the probe footprint may be undetectable.')
        if not self.previous.frame or not self.measured.frame:
            warnings.append('An input has no frame annotation; its coordinates are assumed to follow the supplied common frame/transform.')
        if s.full_3d:
            warnings.append('Normal signs are inferred from the cloud shape/centroid; ambiguous concave or open surfaces need an explicit outward view axis to distinguish dents from bulges reliably.')
        summary = self.summarize(self.analysis)
        registration_ok = (self.registration['status'] in ('provided_transform','empty_measurement') or
                           (self.registration.get('observable_rank',0) >= 3
                            and self.registration.get('inlier_rmse_mm') is not None
                            and self.registration['inlier_rmse_mm'] <= self.settings.tolerance_mm*2))
        if not registration_ok:
            summary['status'] = 'unreliable_alignment'
            summary['damage_percent'] = None
            warnings.append('Alignment quality is insufficient for a reliable damage percentage.')
        rois = [{'id': r.id, 'input': r.data, **self.summarize(r.mask(s.points))} for r in self.rois]
        regions = [dict(region) for region in self.regions]
        if not registration_ok:
            for region in regions:
                region.update(confidence='unreliable_alignment', damage_percent=None, severity_percent=None)
            for roi in rois:
                roi.update(status='unreliable_alignment', damage_percent=None)
        return {'part': self.part, 'frame': self.frame, 'previous': self.previous.path,
                'measured': self.measured.path, 'measured_frame': self.measured.frame,
                'transform_direction': 'measured_to_previous',
                'view_axis': None if s.full_3d else s.axis.tolist(),
                'parameters': asdict(self.settings),
                'surface_scope': 'all_3d_surface_voxels' if s.full_3d else 'outer surface visible along view_axis',
                'area_method': 'surface_voxel_area_estimate' if s.full_3d else 'projected_cell_area_corrected_by_local_normal',
                'registration': self.registration, **summary,
                'rois': rois, 'damage_regions': regions, 'warnings': warnings}


class ComparisonReport:
    def __init__(self, output, settings):
        self.output, self.settings = Path(output), settings

    def save(self, comparisons, plot=True):
        self.output.mkdir(parents=True, exist_ok=True)
        parts = []
        for comparison in comparisons:
            c = comparison; stem = self.output/c.part
            if not c.part or Path(c.part).name != c.part or c.part in ('.', '..'):
                raise ValueError('Part names must be simple file names')
            report = c.report()
            points = c.surface.points[c.analysis]; labels = c.labels[c.analysis]
            known = c.known[c.analysis]
            deviation = np.where(c.observed[c.analysis], c.deviation[c.analysis], np.nan)
            severity = np.where(known, c.severity[c.analysis]*100, np.nan)
            values = np.c_[points, c.COLORS[labels], deviation, severity, labels, known.astype(int)]
            header = '\n'.join(['ply', 'format ascii 1.0', f'comment frame {c.frame}',
                                'comment coordinates in metres; deviation_mm and severity_percent are scalar properties',
                                f'element vertex {len(points)}',
                                *[f'property double {k}' for k in 'xyz'],
                                *[f'property uchar {k}' for k in ['red','green','blue']],
                                *[f'property double {k}' for k in ['deviation_mm','severity_percent']],
                                'property uchar label', 'property uchar assessed', 'end_header'])
            cloud_path, csv_path, plot_path = [Path(str(stem)+suffix) for suffix in ('.ply','.csv','.png')]
            np.savetxt(cloud_path, values, header=header, comments='',
                       fmt=['%.9g']*3+['%d']*3+['%.6g']*2+['%d']*2)
            with csv_path.open('w', newline='') as stream:
                writer = csv.writer(stream); writer.writerow(['x_mm','y_mm','z_mm','deviation_mm','severity_percent','label','assessed'])
                names = {v:k for k,v in c.LABELS.items()}
                for p, d, score, label, assessed in zip(points*1000, deviation, severity, labels, known):
                    writer.writerow([*p, d if np.isfinite(d) else '', score if np.isfinite(score) else '', names[label], bool(assessed)])
            report['artifacts'] = {'scalar_cloud': cloud_path.name, 'points': csv_path.name}
            if plot:
                self.plot(c, plot_path); report['artifacts']['plot'] = plot_path.name
            parts.append(report)
        result = {'schema_version': 1, 'created_at': datetime.now().astimezone().isoformat(),
                  'parameters': asdict(self.settings),
                  'score_definition': 'Area-weighted mean clip((abs(normal deviation)-tolerance)/(full_damage-tolerance),0,1); confirmed missing=1. Unknown cells excluded, with whole-ROI lower/upper bounds.',
                  'baseline_requirement': 'Interpret geometric change as damage only when the previous cloud represents a known healthy/reference shape.',
                  'damage_percent_is_failure_probability': False,
                  'label_codes': PartComparison.LABELS, 'parts': parts}
        path = self.output/'report.json'; path.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
        return path

    @staticmethod
    def plot(comparison, path):
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        c = comparison; fig = plt.figure(figsize=(18, 6))
        report = c.report()
        def percent(value):
            return 'N/A' if value is None else f'{value:.1f}%'
        fig.suptitle(f'Assessed damage {percent(report["damage_percent"])} | '
                     f'Geometric match {percent(report["sync_percent"])} | '
                     f'Coverage {report["coverage_percent"]:.1f}% ({report["status"]})')
        axis = fig.add_subplot(131, projection='3d')
        for points, color, label in [(c.previous.points, 'steelblue', 'Previous'), (c.aligned, 'darkorange', 'Measured (aligned)')]:
            points = points[::max(1, len(points)//5000)]*1000
            axis.scatter(*points.T, s=1, alpha=.35, c=color, label=label)
        axis.set(title=f'{c.part}: scan alignment', xlabel='X [mm]', ylabel='Y [mm]', zlabel='Z [mm]'); axis.legend()
        axis = fig.add_subplot(132, projection='3d' if c.surface.full_3d else None)
        mapped = c.surface.points[c.analysis]*1000 if c.surface.full_3d else c.surface.uv[c.analysis]*1000
        axis.scatter(*mapped.T, c=c.COLORS[c.labels[c.analysis]]/255, s=7)
        for region in sorted(report['damage_regions'],key=lambda r:r['area_mm2'],reverse=True)[:6]:
            centre = np.asarray(region['roi']['centre_mm'])
            label = region['id']
            if c.surface.full_3d:
                axis.text(*centre, label, fontsize=7)
            else:
                axis.annotate(label, centre @ c.surface.basis, fontsize=7)
        axis.set(title='Damage / ROI map (grey = unobserved)',
                 xlabel='X [mm]' if c.surface.full_3d else 'View U [mm]',
                 ylabel='Y [mm]' if c.surface.full_3d else 'View V [mm]')
        if c.surface.full_3d:
            axis.set_zlabel('Z [mm]')
            for roi in c.rois:
                points = c.surface.points[roi.mask(c.surface.points)]*1000
                if len(points):
                    low, high = points.min(0),points.max(0)
                    corners = np.array(list(product(*zip(low,high))))
                    for i in range(8):
                        for j in range(i):
                            if bin(i^j).count('1') == 1:
                                axis.plot(*corners[[i,j]].T,color='black',ls='--',lw=.6)
                    axis.text(*low,roi.id,fontsize=7)
        else:
            from matplotlib.patches import Rectangle
            for roi in c.rois:
                mask = roi.mask(c.surface.points)
                if mask.any():
                    uv = c.surface.uv[mask]*1000; low, high = uv.min(0), uv.max(0)
                    axis.add_patch(Rectangle(low, *(high-low), fill=False, ls='--', lw=.7, color='black'))
                    axis.annotate(roi.id, low, fontsize=7)
            axis.set_aspect('equal')
        axis = fig.add_subplot(133)
        items = report['rois'] or sorted(report['damage_regions'],key=lambda r:r['area_mm2'],reverse=True)[:12]
        positions = np.arange(len(items))
        scores = [item['damage_percent'] if item['damage_percent'] is not None else 0. for item in items]
        axis.barh(positions,scores,color='darkorange',alpha=.8)
        axis.set_yticks(positions,[f'{item["id"]}: {item.get("type","input ROI")}' for item in items],fontsize=8)
        for i,item in enumerate(items):
            axis.text(min(scores[i]+1,90),i,percent(item['damage_percent']),va='center',fontsize=8)
        axis.invert_yaxis(); axis.set_xlim(0,105); axis.set_xlabel('Assessed geometric damage [%]')
        axis.set_title('ROI / largest detected regions'); axis.grid(axis='x',alpha=.3)
        fig.tight_layout(); fig.savefig(path, dpi=140); plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--previous', '--reference', type=Path)
    parser.add_argument('--measured', type=Path)
    parser.add_argument('--part', default='part')
    parser.add_argument('--roi', type=Path)
    parser.add_argument('--manifest', type=Path, help='JSON parts list; paths are relative to this manifest')
    parser.add_argument('--axis', nargs=3, type=float, help='Select one outward surface view; default compares the entire 3-D cloud')
    parser.add_argument('--initial-transform', '--scene-pose', type=Path,
                        help='Measured-to-previous rigid 4x4 matrix (metres), or the scene_truth.json captured with this scan')
    parser.add_argument('--units', choices=['m','mm'], default='m')
    parser.add_argument('--no-align', action='store_true', help='Keep the supplied frame/transform; useful with calibrated poses')
    parser.add_argument('--complete-coverage', action='store_true', help='Absent samples become missing candidates, not confirmed loss')
    parser.add_argument('--no-plot', action='store_true')
    parser.add_argument('--output', type=Path, default=Path('pointcloud_comparison')/datetime.now().strftime('%Y%m%d_%H%M%S'))
    for name, default in [('cell-mm',1.), ('tolerance-mm',.5), ('full-damage-mm',5.),
                          ('match-radius-mm',1.5), ('max-deviation-mm',20.), ('registration-radius-mm',10.)]:
        parser.add_argument('--'+name, type=float, default=default)
    args = parser.parse_args()
    try:
        settings = Settings(**{k:getattr(args,k) for k in ['cell_mm','tolerance_mm','full_damage_mm','match_radius_mm','max_deviation_mm','registration_radius_mm']}, align=not args.no_align)
    except ValueError as error:
        parser.error(str(error))
    if args.manifest:
        entries = json.loads(args.manifest.read_text())['parts']; base = args.manifest.resolve().parent
    else:
        if args.previous is None or args.measured is None:
            parser.error('--previous and --measured, or --manifest, are required')
        entries = [{'part':args.part, 'previous':str(args.previous.resolve()), 'measured':str(args.measured.resolve()),
                    'roi':str(args.roi.resolve()) if args.roi else None,
                    'axis':args.axis, 'initial_transform':str(args.initial_transform.resolve()) if args.initial_transform else None}]
        base = Path.cwd()
    comparisons = []
    for entry in entries:
        part = str(entry['part'])
        if any(c.part == part for c in comparisons):
            raise ValueError('Manifest part names must be unique; use view suffixes for multiple axes')
        previous = PointCloud.load(base/entry['previous'], entry.get('units',args.units))
        measured = PointCloud.load(base/entry['measured'], entry.get('units',args.units))
        roi = entry.get('roi',args.roi.resolve() if args.roi else None); roi = base/roi if isinstance(roi,str) else roi
        initial = entry.get('initial_transform',args.initial_transform.resolve() if args.initial_transform else None)
        initial = base/initial if isinstance(initial,str) else initial
        transform = RigidRegistration.transform(initial, measured.frame, previous.frame or 'base_link')
        rois = ROI.load(roi, part, previous.frame or 'base_link', transform if initial is not None else None, measured.frame)
        part_settings = Settings(**{**asdict(settings), **entry.get('settings',{})})
        comparison = PartComparison(part, previous, measured, part_settings, entry.get('axis',args.axis), rois, initial,
                                    entry.get('coverage_complete',args.complete_coverage))
        comparisons.append(comparison)
        result = comparison.report()
        def display(value):
            return 'N/A' if value is None else f'{value:.2f}%'
        print(f'{part}: sync={display(result["sync_percent"])}; damage={display(result["damage_percent"])} '
              f'(assessed surface); coverage={result["coverage_percent"]:.2f}%; {result["status"]}')
    if not comparisons:
        parser.error('No parts to compare')
    print(ComparisonReport(args.output,settings).save(comparisons,plot=not args.no_plot))


if __name__ == '__main__':
    main()
