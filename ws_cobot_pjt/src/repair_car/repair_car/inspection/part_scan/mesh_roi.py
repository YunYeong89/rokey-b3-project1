"""Offline normal-following ROI evidence; shared default thresholds, clustering and JSON format."""
from types import SimpleNamespace

import numpy as np

from repair_car.config import load
from repair_car.inspection.part_scan.cloud_roi import CloudRoi
from repair_car.inspection.part_scan.crack_roi import CrackRoi, DoorCrackRoi
from repair_car.inspection.part_scan.pose_filter import PoseHeightFilter
from repair_car.inspection.part_scan.staged_roi import SurveyRoi


class MeshRoi(SurveyRoi):
    def __init__(self, records, metadata, reference=None):
        config = load('robot')
        scan = config['real_scan']
        defaults = scan['default_scan']
        self.settings = dict(metadata.get('roi_settings', defaults['roi']))
        self.threshold = metadata.get('contact_threshold_n', defaults['contact_threshold_n'])
        self.settings.update(contact_threshold_n=self.threshold,
                             reference='Planned ball-centre path; displacement projected onto the surface normal',
                             parameters_source='saved plan' if 'roi_settings' in metadata else 'current robot.yaml')
        self.records = rows = np.asarray([[r[0], *r[3:6], r[18], *r[9:12], r[1]] for r in records], float)
        self.position = np.asarray([r[66:69] for r in records], float)
        self.wrench = np.asarray([r[12:15] for r in records], float)  # BASE reaction (N)
        target = np.asarray([r[69:72] for r in records], float)
        self.depth = np.einsum('ij,ij->i', target - self.position, rows[:, 5:8])
        self.residual = np.full(len(rows), np.nan)
        self.force_limit = np.full(len(rows), np.nan)
        self.depth_limit = np.full(len(rows), np.nan)
        self.flags = {key: np.zeros(len(rows), bool) for key in ('geometry', 'force', 'contact_loss')}
        self.regions, self.limits, self.damage_samples = [], [], []
        stroke = np.array([r[2] == 'stroke' for r in records])
        raw_loss = stroke & (rows[:, 4] <= self.threshold)
        self.excluded_loss = np.zeros(len(rows), bool)
        self.paired = np.zeros(len(rows), bool)
        self.confirmed = np.zeros(len(rows), bool)
        self.crack_crossings = np.zeros(len(rows), bool)
        cloud = None
        if reference is not None:
            settings = metadata.get('settings', {})
            cloud_settings = {**config['bonnet_roi_cloud'], **(metadata.get('roi_cloud_settings') or {})}
            # The crack criteria always as robot.yaml now: a saved plan's older ones lack the across-axis evidence.
            cloud_settings['crack'] = config['bonnet_roi_cloud']['crack']
            if metadata.get('rescan_of'):  # an ROI rescan: stricter (bonnet_roi_cloud.rescan)
                cloud_settings['crack'] = {**cloud_settings['crack'], **config['bonnet_roi_cloud']['rescan']['crack']}
            cloud = CloudRoi(reference, settings.get('ball_radius_mm', config['probe']['tip_radius_mm']),
                             cloud_settings,
                             settings.get('pointcloud', scan['mesh_height']['pointcloud']))
            support, usable, roi_surface = cloud.sample(self.position)
            depth_from_cloud = support - self.position[:, 2]
            velocity = np.asarray([r[72:74] for r in records], float)
            heading = cloud.headings(velocity)
            stable = cloud.steady(velocity, heading, metadata['speed_mm_s'])
            raster = np.zeros(len(rows), bool)
            self.excluded_loss = raw_loss & ~roi_surface
            self.depth.fill(np.nan)
            self.settings['reference'] = 'Nominal point-cloud ball support and local same-direction stroke baseline'
        groups = np.cumsum(np.r_[True, (np.diff(rows[:, 8]) != 0) | (np.diff(rows[:, 0]) > .25)
                                | (stroke[1:] != stroke[:-1])])
        for path in np.unique(rows[stroke, 8]):
            at = np.flatnonzero(stroke & (rows[:, 8] == path))
            if len(at) < 3:
                continue
            force = rows[at, 4]
            baseline = float(np.median(force))
            residual = force - baseline
            # mesh-height (one fixed axis): the depth corrected for the posture wobble (PoseHeightFilter).
            pose = PoseHeightFilter(rows[at[0], 5:8], self.threshold) if np.ptp(rows[at, 5:8], 0).max() < 1e-9 else None
            if pose is not None and cloud is None:
                self.depth[at] = pose.compliance(pose.reference(self.position[at], target[at]), force,
                                                 self.position[at])
            depth = self.depth[at]
            if cloud is not None:
                axes = heading[at] // 2
                main_axis = np.bincount(axes[stable[at]], minlength=2).argmax()
                raster[at] = stable[at] & (axes == main_axis)
                observed = (depth_from_cloud[at] if pose is None else
                            pose.compliance(depth_from_cloud[at], force, self.position[at]))
                depth, residual = cloud.residuals(self.position[at], np.c_[observed, force], velocity[at],
                                                  usable[at], metadata['speed_mm_s'], self.settings['link_mm']).T
                self.depth[at] = depth
                if not np.isfinite(depth).any():
                    continue
                baseline = float(np.nanmedian(force - residual))
                force_limit = cloud.settings['confirm_force_drop_n']
                depth_limit = cloud.settings['confirm_depth_mm']
                flags = dict(force=residual <= -force_limit, geometry=depth >= depth_limit,
                             contact_loss=raw_loss[at] & roi_surface[at] &
                             ((residual <= -force_limit) | (depth >= depth_limit)))
            else:
                force_limit = max(self.settings['force_min_n'], self.settings['noise_sigma'] * 1.4826 *
                                  np.nanmedian(np.abs(residual)))
                depth_limit = max(self.settings['depth_min_mm'], self.settings['noise_sigma'] * 1.4826 *
                                  np.nanmedian(np.abs(depth - np.nanmedian(depth))))
                flags = dict(force=np.abs(residual) > force_limit, geometry=np.abs(depth) > depth_limit,
                             contact_loss=raw_loss[at])
            self.residual[at], self.force_limit[at], self.depth_limit[at] = residual, force_limit, depth_limit
            axis = rows[at, 5:8].mean(0)
            axis /= np.linalg.norm(axis)
            self.limits.append(dict(path=int(path), axis=axis.tolist(), force_n=float(force_limit),
                                    depth_mm=float(depth_limit), force_baseline_n=baseline,
                                    baseline_source=('local same-direction reaction' if cloud else
                                                     'median recorded normal reaction'),
                                    target_force_n=metadata.get('target_normal_force_n'),
                                    pose_filter_stiffness_n_mm=(None if pose is None or not np.isfinite(pose.stiffness)
                                                                else round(pose.stiffness, 2))))
            u = np.array([1., 0., 0.]) if abs(axis[0]) < .9 else np.array([0., 1., 0.])
            u -= axis * (u @ axis)
            u /= np.linalg.norm(u)
            grid = SimpleNamespace(u=u, v=np.cross(axis, u))
            boundaries = np.r_[0, np.flatnonzero(np.diff(groups[at])) + 1, len(at)]
            for reason, flag in flags.items():
                kept = np.zeros(len(at), bool)
                for first, last in zip(boundaries[:-1], boundaries[1:]):
                    edges = np.flatnonzero(np.diff(np.r_[False, flag[first:last], False]))
                    for low, high in zip(edges[::2], edges[1::2]):
                        run = at[first + low:first + high]
                        # Commanded path travel excludes stationary pressing, settling and resume holds.
                        travel = np.linalg.norm(np.diff(target[run], axis=0), axis=1).sum()
                        if len(run) >= 3 and travel >= self.settings['min_run_mm']:
                            kept[first + low:first + high] = True
                self.flags[reason][at[kept]] = True
                if kept.any() and cloud is None:
                    self.regions += self.components(rows[at[kept], 1:4], depth[kept], residual[kept],
                                                    axis, grid, reason)
        if cloud is None:
            self.regions = self.merge(self.regions)
            for region in self.regions:
                region['type'] = 'scan_candidate'
            if stroke.any() and metadata.get('part') == 'door' and metadata.get('controller') == 'mesh-height':
                self.door_crack_regions(records, target, stroke, groups, metadata['speed_mm_s'], config['door_crack_roi'])
        else:
            roi_strokes = roi_surface & raster
            self.dent_regions(cloud, support, heading, groups, roi_strokes)
            self.crack_regions(cloud, support, heading, groups, roi_strokes)
            self.settings['cloud_filter'] = dict(**cloud.settings, stroke_samples=int(stroke.sum()),
                usable_surface_samples=int((stroke & usable).sum()), compared_samples=int(np.isfinite(self.depth).sum()),
                eligible_roi_samples=int((stroke & roi_surface).sum()),
                raw_contact_loss_samples=int(raw_loss.sum()), excluded_by_surface=int(self.excluded_loss.sum()),
                kept_contact_loss_samples=int(self.flags['contact_loss'].sum()),
                paired_samples=int(self.paired.sum()), cross_pass_samples=int(self.confirmed.sum()),
                crack_crossings=int(self.crack_crossings.sum()))
        self.limit = max((item['force_n'] for item in self.limits), default=self.settings['force_min_n'])
        if cloud is not None and metadata.get('rescan_of') and config['bonnet_roi_cloud']['rescan']['confirmed_only']:
            self.damage_samples = [(region, points) for region, points in self.damage_samples if region['confirmed']]
            self.regions = [region for region, _ in self.damage_samples]
        for number, region in enumerate(self.regions, 1):
            region['id'] = f'roi_{number:03d}'

    def door_crack_regions(self, records, target, stroke, groups, speed, settings):
        axis = self.records[stroke, 5:8][0]
        velocity = np.asarray([row[72:75] for row in records], float)
        self.settings['door_crack'] = settings
        detector = DoorCrackRoi(settings)
        for at, values, length, width, confirmed, lanes in detector.detect(
                self.position, target, axis, self.records[:, 4], self.wrench, velocity,
                self.records[:, 8], groups, stroke, speed, self.threshold):
            self.crack_crossings[at] = True
            points = self.position[at]
            region = self.region(points, axis, ['force'], max(0., float(values[:, 0].max())), float(values[:, 1].max()))
            region.update(type='crack', confirmed=confirmed, force_samples=len(at), raster_rows=int(lanes),
                          length_mm=length, width_mm=width,
                          force_contrast_xyz_n=np.abs(values[:, 3:]).max(0).round(3).tolist(),
                          crossing_bounds_yz_mm=np.r_[points[:, 1:].min(0), points[:, 1:].max(0)].tolist())
            self.regions.append(region)
            self.damage_samples.append((region, points))

    def dent_regions(self, cloud, support, heading, groups, roi_surface):
        depth = np.where(roi_surface, self.depth, np.nan)
        self.paired = cloud.paired_strokes(self.position, depth, self.residual, heading, groups)
        at = np.flatnonzero(self.paired)
        if not len(at):
            return
        step = self.settings['link_mm']
        labels = cloud.cross_pass_labels(self.position[at], heading[at], step)
        self.confirmed[at[labels > 0]] = True
        for group in np.unique(labels[labels > 0]):
            self.add_defect('dent', at[labels == group], support, cloud, confirmed=True)
        pending = at[(labels == 0) & ~self.within_confirmed(self.position[at, :2])]
        if not len(pending):
            return
        labels = self.component_labels(self.position[pending], cloud.grid)
        for group in np.unique(labels):
            selected = pending[labels == group]
            xy = self.position[selected, :2]
            if np.all(np.ptp(xy, axis=0) >= step / 2):
                self.add_defect('dent', selected, support, cloud, confirmed=False)

    def crack_regions(self, cloud, support, heading, groups, usable):
        settings = cloud.settings['crack']
        detector = CrackRoi(settings)
        # Noise from the reference cloud: where the nominal surface turns under the ball (character-line flanks) the
        # ball's own geometry moves the height and the reaction, so no crossing is taken there.
        quiet = usable & (cloud.normal_change_at(self.position) <= settings['max_normal_change_deg'])
        axis = self.records[:, 5:8]
        tangential = self.wrench - axis * np.einsum('ij,ij->i', self.wrench, axis)[:, None]
        peaks, contrasts = detector.peaks(self.position, support - self.position[:, 2], self.records[:, 4],
                                           tangential, heading, groups, quiet, self.threshold)
        keep = ~self.within_confirmed(self.position[peaks, :2])
        peaks, contrasts = peaks[keep], contrasts[keep]
        for own, length, width, confirmed in detector.traces(self.position[peaks], heading[peaks], contrasts[:, 1]):
            at = peaks[own]
            self.crack_crossings[at] = True
            self.add_defect('crack', at, support, cloud, confirmed=confirmed,
                            contrast=contrasts[own], length_mm=length, width_mm=width)

    def within_confirmed(self, xy):
        covered = np.zeros(len(xy), bool)
        for region in self.regions:
            if region['confirmed']:
                low, high = np.asarray(region['bounds_mm']).reshape(2, 3)
                covered |= np.all((xy >= low[:2]) & (xy <= high[:2]), axis=1)
        return covered

    def add_defect(self, kind, at, support, cloud, *, confirmed, contrast=None, **dimensions):
        points = np.c_[self.position[at, :2], support[at] - cloud.radius]
        depth, force = contrast.T if contrast is not None else (self.depth[at], -self.residual[at])
        evidence = ['geometry', 'force']
        if self.flags['contact_loss'][at].any():
            evidence.append('contact_loss')
        region = self.region(points, cloud.grid.axis, evidence, depth.max(), -force.max())
        region.update(type=kind, confirmed=confirmed, force_samples=len(at), **dimensions)
        self.regions.append(region)
        self.damage_samples.append((region, self.position[at]))
