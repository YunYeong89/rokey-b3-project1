"""Gazebo performance coupon: nominal planning surface and offline evaluation.

The controller receives only the nominal plane. Mesh heights and ball envelopes are evaluation data.
STL coordinates are millimetres; exported clouds use metres in base_link.
"""
from copy import copy
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np

if __package__:
    from .configs import PACKAGE, load
    from .real_vehicle import RealVehicle, clip, read_stl
else:
    from configs import PACKAGE, load
    from real_vehicle import RealVehicle, clip, read_stl


class PerformanceSample(RealVehicle):
    kind = 'performance'

    def __init__(self, source=None):
        self.spec = load('vehicles')['performance']
        self.source = Path(source) if source is not None else PACKAGE / self.spec['mesh']
        self.source_hash = hashlib.sha256(self.source.read_bytes()).hexdigest()
        # The supplied coupon is Y-up; this is a proper +90 degree rotation about X.
        self.triangles = read_stl(self.source) @ np.array([[1., 0, 0], [0, 0, 1], [0, -1, 0]])
        low, high = self.triangles.reshape(-1, 3).min(0), self.triangles.reshape(-1, 3).max(0)
        self.triangles -= [(low[0]+high[0])/2, (low[1]+high[1])/2, low[2]]
        self.size, self.damage = high-low, []
        self.parts = np.full(len(self.triangles), 'box', dtype=object)

    def placement(self):
        return list(self.spec['pose_mm_deg'])

    def part_colors(self):
        return {'box': self.spec['color']}

    def nominal(self):
        """A healthy rectangular solid of the same external dimensions, without using defect locations."""
        model = copy(self)
        x, y, z = self.size
        vertices = np.array([[-x/2,-y/2,0], [x/2,-y/2,0], [x/2,y/2,0], [-x/2,y/2,0],
                             [-x/2,-y/2,z], [x/2,-y/2,z], [x/2,y/2,z], [-x/2,y/2,z]])
        faces = [[0,2,1],[0,3,2],[4,5,6],[4,6,7],[0,1,5],[0,5,4],
                 [1,2,6],[1,6,5],[2,3,7],[2,7,6],[3,0,4],[3,4,7]]
        model.triangles = vertices[faces]
        model.parts = np.full(len(faces), 'box', dtype=object)
        return model

    def reference(self, pose, spacing):
        x, y = [np.linspace(-side/2, side/2, int(np.ceil(side/spacing))+1) for side in self.size[:2]]
        xx, yy = np.meshgrid(x, y)
        points = np.c_[xx.ravel(), yy.ravel(), np.full(xx.size, self.size[2])]
        points = points @ pose[:3,:3].T + pose[:3,3]
        return SimpleNamespace(part='box', points=points, path=self.source)

    def planning_surface(self, roi=None, regions=None):
        """Healthy nominal mesh for mesh-height; defect triangles are never planning input."""
        model = self.nominal()
        regions = [roi] if roi is not None else regions
        if regions:
            rectangles = np.asarray(regions, float).reshape(-1, 2, 2)
            low, high = rectangles[:, 0], rectangles[:, 1]
            if np.any(low < -self.size[:2] / 2) or np.any(high > self.size[:2] / 2):
                raise ValueError('--roi lies outside the performance coupon')
            for axis in (0, 1):
                for value in np.unique(rectangles[:, :, axis]):
                    model.triangles = clip(model.triangles, axis, value)
            centres = model.triangles.mean(1)
            target = np.any(np.all((centres[:, None, :2] >= low) &
                                  (centres[:, None, :2] <= high), axis=2), axis=1)
            target &= np.all(np.isclose(model.triangles[:, :, 2], self.size[2]), axis=1)
            model.parts = np.where(target, 'box', 'other')
        return model

    @staticmethod
    def planning_regions(settings, region=None, roi=None):
        if roi is not None:
            return [roi]
        names = ('crack', 'dent') if region in (None, 'all') else (region,)
        return [bounds for name in names for bounds in settings['mesh_height_regions_mm'][name]]

    def top_triangles(self):
        return self.triangles[self.normals()[:,2] > 1e-7]

    def heights(self, xy):
        """Vertical upper intersections with source triangles; NaN outside the coupon."""
        xy = np.asarray(xy, float).reshape(-1,2)
        height = np.full(len(xy), -np.inf)
        for triangle in self.top_triangles():
            inside = np.all((xy >= triangle[:,:2].min(0)-1e-8) &
                            (xy <= triangle[:,:2].max(0)+1e-8), axis=1)
            indices = np.flatnonzero(inside)
            if not len(indices):
                continue
            a, b, c = triangle
            uv = (xy[indices]-a[:2]) @ np.linalg.inv(np.stack([b[:2]-a[:2], c[:2]-a[:2]]))
            keep = (uv >= -1e-7).all(1) & (uv.sum(1) <= 1+1e-7)
            indices, uv = indices[keep], uv[keep]
            z = a[2]+uv @ np.array([b[2]-a[2], c[2]-a[2]])
            height[indices] = np.maximum(height[indices], z)
        return np.where(np.isfinite(height), height, np.nan)



class PerformanceReport:
    """Reports in a coupon-fixed frame; raw geometry is never re-centred over damaged samples."""
    def __init__(self, model, pose, reference, directory, name, radius, threshold, settings, figures=None, sigma_s=.02,
                 plot_force=True):
        self.model, self.pose, self.reference = model, pose, reference
        self.directory, self.name = Path(directory), name
        self.radius, self.threshold, self.settings = radius, threshold, settings
        from .part_scan.gaussian import GaussianSmoother
        self.figures, self.gaussian = figures, GaussianSmoother(sigma_s)
        self.plot_force = plot_force
        self.directory.mkdir(parents=True, exist_ok=True)

    def subplots(self, *args, **kwargs):
        import matplotlib.pyplot as plt
        return self.figures.subplots(*args, **kwargs) if self.figures else plt.subplots(*args, **kwargs)

    def close_figure(self, fig, path):
        if self.figures:
            return self.figures.path
        import matplotlib.pyplot as plt
        fig.tight_layout(); fig.savefig(path, dpi=140); plt.close(fig)
        return path

    def local(self, points):
        return (np.asarray(points)-self.pose[:3,3]) @ self.pose[:3,:3]

    def save_plan(self, plan, roi):
        root = self.directory/self.name
        root.mkdir(exist_ok=True)
        value = dict(source_mesh=str(self.model.source),source_hash=self.model.source_hash,probe_radius_mm=self.radius,
                     pose_base_mm=self.pose.tolist(),roi_local_mm=roi,
                     planning_reference='healthy nominal plane only',pitch_mm=plan.scan['pitch_mm'],
                     target_speed_mm_s=plan.scan['speed_mm_s'],force_n=plan.scan['force_n'],
                     point_decimation=plan.scan.get('point_decimation', 1),
                     stiffness=plan.scan['stiffness'],
                     paths=[dict(axis=axis.tolist(),points_base_mm=points.tolist()) for points,axis in plan.paths])
        path = root/'plan.json'
        path.write_text(json.dumps(value,indent=2)+'\n')
        return path

    def truth_grid(self):
        step = self.settings['truth_step_mm']
        x, y = [np.linspace(-s/2,s/2,int(np.ceil(s/step))+1) for s in self.model.size[:2]]
        xx, yy = np.meshgrid(x,y)
        height = self.model.heights(np.c_[xx.ravel(),yy.ravel()]).reshape(xx.shape)
        # Sampled spherical envelope: reference is the sphere's lowest point on a plane.
        dx, dy = x[1]-x[0], y[1]-y[0]
        envelope = np.full_like(height,-np.inf)
        r = self.radius
        padded = np.pad(height,((int(np.ceil(r/dy)),)*2,(int(np.ceil(r/dx)),)*2),constant_values=-np.inf)
        ry, rx = int(np.ceil(r/dy)), int(np.ceil(r/dx))
        for j in range(-ry,ry+1):
            for i in range(-rx,rx+1):
                distance2 = (i*dx)**2+(j*dy)**2
                if distance2 <= r*r+1e-12:
                    candidate = padded[ry+j:ry+j+len(y),rx+i:rx+i+len(x)] + np.sqrt(max(0.,r*r-distance2))-r
                    envelope = np.maximum(envelope,candidate)
        return x,y,height,envelope

    def save(self, scraper, completed, plan):
        self.force_target = scraper.scan['force_n']
        from scipy.ndimage import label, map_coordinates
        from .pointcloud_damage import ComparisonReport, PartComparison, PointCloud, Settings
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt

        root = self.directory/self.name
        root.mkdir(exist_ok=True)
        points = np.asarray(scraper.points, float).reshape(-1,4)
        nominal = self.reference.points
        local = self.local(points[:,:3]*1000)
        raw_local = local.copy()
        point_samples = np.asarray(scraper.point_samples, float).reshape(-1, 2)
        filtered_points = points.copy()
        filtered_points[:, :3] = self.gaussian.apply(point_samples[:, 0], points[:, :3], point_samples[:, 1])
        filtered_points[:, 3] = self.gaussian.apply(point_samples[:, 0], points[:, 3], point_samples[:, 1])
        filtered_local = self.local(filtered_points[:, :3] * 1000)
        tolerance, full = self.settings['tolerance_mm'], self.settings['full_damage_mm']
        anchor = np.asarray(self.settings['calibration_roi_mm']).reshape(2,2)
        healthy = (np.all((local[:,:2]>=anchor[0]) & (local[:,:2]<=anchor[1]),axis=1)
                   & (np.abs(points[:,3]-self.force_target)<.25))
        chosen = local[healthy]
        bias,calibrated = 0.,False
        if len(chosen)>=20 and np.all(np.abs(self.model.heights(chosen[:,:2])-self.model.size[2])<tolerance):
            keys = np.floor(chosen[:,:2]/self.settings['analysis_cell_mm']).astype(int)
            _,groups = np.unique(keys,axis=0,return_inverse=True)
            if groups.max()>=5:
                bias = float(np.median([np.median(chosen[groups==k,2]) for k in np.unique(groups)])-self.model.size[2])
                calibrated = True
        calibration = dict(roi_local_mm=anchor.ravel().tolist(),offset_mm=bias,
                           applied=calibrated,eligible_samples=int(healthy.sum()),
                           method='Median of spatial cell heights in the declared healthy ROI; only a Z offset')
        local[:,2] -= bias
        filtered_local[:, 2] -= bias
        corrected = points.copy(); corrected[:,:3] -= bias/1000*self.pose[:3,2]
        filtered_points[:, :3] -= bias / 1000 * self.pose[:3, 2]
        np.savetxt(root/'point_samples.csv', np.c_[point_samples, corrected, filtered_points], delimiter=',',
                   header='t_s,path,raw_calibrated_x_m,raw_calibrated_y_m,raw_calibrated_z_m,raw_force_n,gaussian_calibrated_x_m,gaussian_calibrated_y_m,gaussian_calibrated_z_m,gaussian_force_n',comments='')
        comparison = PartComparison('box',PointCloud(nominal/1000,'base_link'),
                                    PointCloud(corrected[:,:3],'base_link'),
                                    Settings(cell_mm=self.settings['analysis_cell_mm'],tolerance_mm=tolerance,
                                             full_damage_mm=full, match_radius_mm=min(plan.scan['pitch_mm']*.45,
                                                                                    self.settings['analysis_cell_mm']*.75),
                                             align=False),axis=[0,0,1])
        filtered_comparison = PartComparison('box_gaussian', PointCloud(nominal/1000,'base_link'),
                                             PointCloud(filtered_points[:, :3], 'base_link'), comparison.settings, axis=[0,0,1])
        comparison_report = ComparisonReport(root,comparison.settings)
        comparison_report.save([comparison, filtered_comparison], plot=not bool(self.figures), export_cloud=False)
        if self.figures:
            for item in (comparison, filtered_comparison):
                comparison_report.plot(item, None, figure=self.figures.next())
        truth = self.truth_grid()
        x,y,height,envelope = truth
        xx,yy = np.meshgrid(x,y)
        coords = np.vstack([(local[:,1]-y[0])/(y[1]-y[0]),(local[:,0]-x[0])/(x[1]-x[0])])
        inside = np.all((local[:,:2]>= [x[0],y[0]]) & (local[:,:2]<= [x[-1],y[-1]]),axis=1)
        gt = map_coordinates(height,coords,order=1,mode='constant',cval=np.nan)
        observable = map_coordinates(envelope,coords,order=1,mode='constant',cval=np.nan)
        valid = inside & np.isfinite(gt) & np.isfinite(observable)
        error = local[:,2]-observable
        depth = self.model.size[2]-local[:,2]
        filtered_depth = self.model.size[2] - filtered_local[:, 2]
        np.savetxt(root/'geometry.csv',np.c_[local,raw_local[:,2],depth,gt,observable,error,valid,filtered_local,filtered_depth],delimiter=',',
                   header='x_mm,y_mm,calibrated_z_mm,raw_z_mm,depth_mm,mesh_height_mm,ball_envelope_height_mm,envelope_error_mm,inside,gaussian_x_mm,gaussian_y_mm,gaussian_z_mm,gaussian_depth_mm',comments='')
        fig,axes = self.subplots(1,4,figsize=(20,5))
        for axis,z,title,values in zip(axes,(height,envelope,None,None),
                                      ('Mesh ground truth','Ball-observable ground truth','Raw depth','Gaussian depth'),
                                      (local,local,local,filtered_local)):
            if z is None:
                dots = axis.scatter(values[:,0],values[:,1],c=self.model.size[2]-values[:,2],s=2,vmin=0,vmax=full,cmap='magma')
            else:
                dots = axis.pcolormesh(xx,yy,self.model.size[2]-z,shading='auto',vmin=0,vmax=full,cmap='magma')
            fig.colorbar(dots,ax=axis,label='Depth [mm]')
            axis.set(title=title,xlabel='Coupon X [mm]',ylabel='Coupon Y [mm]',aspect='equal')
            axis.set_xlim(-self.model.size[0]/2,self.model.size[0]/2)
            axis.set_ylim(-self.model.size[1]/2,self.model.size[1]/2)
        self.close_figure(fig, root/'depth.png')

        # Grid cells give equal spatial weight despite slower motion / dwell at corners.
        step = self.settings['analysis_cell_mm']
        keys = np.floor((local[:,:2]+self.model.size[:2]/2)/step).astype(int)
        groups = {}
        for k in np.flatnonzero(valid):
            groups.setdefault(tuple(keys[k]),[]).append(k)
        cells = np.array([np.median(local[v],axis=0) for v in groups.values()]).reshape(-1,3)
        filtered_cells = np.array([np.median(filtered_local[v],axis=0) for v in groups.values()]).reshape(-1,3)
        cell_coords = np.vstack([(cells[:,1]-y[0])/(y[1]-y[0]),(cells[:,0]-x[0])/(x[1]-x[0])])
        expected = map_coordinates(envelope,cell_coords,order=1,mode='constant',cval=np.nan)
        gt_depth = self.model.size[2]-expected
        detected = self.model.size[2]-cells[:,2] > tolerance
        positive = gt_depth > tolerance
        tp,fp,fn = [int(mask.sum()) for mask in (detected & positive,detected & ~positive,~detected & positive)]
        def ratio(a,b):
            return float(a/b) if b else None
        metrics = dict(completed=completed,source_mesh=str(self.model.source),source_hash=self.model.source_hash,
                       pose_base_mm=self.pose.tolist(),probe_radius_mm=self.radius,
                       truth_step_mm=self.settings['truth_step_mm'],
                       truth_envelope_method='sampled spherical dilation; no sub-grid accuracy claim',
                       nominal_top_mm=float(self.model.size[2]),target_press_base_z_n=-self.force_target,
                       height_calibration=calibration,
                       coverage=comparison.report(),
                       measured_contact_samples=len(points),assessed_spatial_cells=len(cells),
                       envelope_rmse_mm=float(np.sqrt(np.mean((cells[:,2]-expected)**2))) if len(cells) else None,
                       mesh_rmse_mm=float(np.sqrt(np.mean((local[valid,2]-gt[valid])**2))) if valid.any() else None,
                       raw_envelope_rmse_mm=float(np.sqrt(np.mean((raw_local[valid,2]-observable[valid])**2))) if valid.any() else None,
                       precision_assessed=ratio(tp,tp+fp),recall_assessed=ratio(tp,tp+fn),
                       true_positive_cells=tp,false_positive_cells=fp,false_negative_cells=fn,
                       detection_scope='Only sampled cells. Unmeasured defects remain unassessed, not false negatives.',
                       warnings=['Geometric defect types are candidates without labelled ground-truth ROI annotations.',
                                 'Scan pitch and the probe footprint limit the smallest detectable defect.'])
        metrics['gaussian'] = dict(sigma_s=self.gaussian.sigma, applied_to='offline measured XYZ and force, separately per path',
                                  controller_feedback_filtered=False, calibration='same raw-derived Z offset in both views',
                                  envelope_rmse_mm=float(np.sqrt(np.mean((filtered_cells[:, 2]-expected)**2))) if len(cells) else None)
        if not calibrated:
            metrics['warnings'].append('Healthy calibration ROI was not sufficiently sampled; uncorrected height offsets may affect damage scores.')
        # Source-mesh connected depth regions are evaluation ROIs, not controller input.
        labels,count = label(self.model.size[2]-height > tolerance,structure=np.ones((3,3)))
        rois = []
        cell_label = map_coordinates(labels,cell_coords,order=0,mode='constant',cval=0)
        for index in range(1,count+1):
            row,col = np.nonzero(labels==index)
            own = cell_label==index
            measured_depth = self.model.size[2]-cells[own,2]
            gaussian_depth = self.model.size[2] - filtered_cells[own, 2]
            observable_depth = self.model.size[2]-envelope[row,col]
            rois.append(dict(id=f'gt_{index:03d}',type='geometric_defect',frame='performance',
                             bounds_mm=[float(x[col.min()]),float(y[row.min()]),float(x[col.max()]),float(y[row.max()])],
                             mesh_max_depth_mm=float((self.model.size[2]-height[row,col]).max()),
                             observable_max_depth_mm=float(observable_depth.max()),
                             observable_at_probe_resolution=bool((observable_depth>tolerance).any()),
                             area_mm2=float(len(row)*(x[1]-x[0])*(y[1]-y[0])),
                             assessed_cells=int(own.sum()),status='assessed' if own.any() else 'unobserved',
                             measured_max_depth_mm=float(measured_depth.max()) if own.any() else None,
                             gaussian_measured_max_depth_mm=float(gaussian_depth.max()) if own.any() else None,
                             gaussian_detected=bool((gaussian_depth>tolerance).any()) if own.any() else None,
                             gaussian_damage_percent=float(100*np.mean(np.clip((np.abs(gaussian_depth)-tolerance)/(full-tolerance),0,1))) if own.any() else None,
                             detected=bool((measured_depth>tolerance).any()) if own.any() else None,
                             damage_percent=float(100*np.mean(np.clip((np.abs(measured_depth)-tolerance)/(full-tolerance),0,1))) if own.any() else None))
        metrics['ground_truth_rois'] = rois
        fig,axis = self.subplots(figsize=(9,max(4,len(rois)*.3)))
        scores = [r['damage_percent'] or 0. for r in rois]
        gaussian_scores = [r['gaussian_damage_percent'] or 0. for r in rois]
        axis.barh(np.arange(len(rois))-.18,scores,height=.35,color='steelblue',label='Raw')
        axis.barh(np.arange(len(rois))+.18,gaussian_scores,height=.35,color='darkorange',label='Gaussian')
        axis.set_yticks(np.arange(len(rois)),[r['id'] for r in rois])
        for i,roi in enumerate(rois):
            text = 'unobserved' if roi['damage_percent'] is None else f'{roi["damage_percent"]:.1f}%'
            axis.text(min(scores[i]+1,90),i,text,va='center',fontsize=8)
        axis.set(xlim=(0,105),xlabel='Assessed geometric damage [%]',title='Source-mesh defect ROIs')
        axis.legend(); axis.invert_yaxis(); self.close_figure(fig, root/'roi.png')
        self.feed_target = plan.scan['speed_mm_s']
        metrics['target_speed_mm_s'] = self.feed_target
        metrics.update(self.telemetry(scraper,root,truth))
        if self.figures:
            metrics['figure'] = self.figures.path.name
        (root/'performance.json').write_text(json.dumps(metrics,indent=2,allow_nan=False)+'\n')
        return root/'performance.json'

    def telemetry(self, scraper, root, truth):
        import matplotlib.pyplot as plt
        from scipy.ndimage import map_coordinates
        from .rotations import rotation_zyz
        records = np.asarray(scraper.records,float).reshape(-1,9)
        if not len(records):
            return dict(contact_retention_percent=None,force_rmse_n=None,feed_median_mm_s=None)
        measured = np.asarray(scraper.telemetry,float)
        rotations = np.array([rotation_zyz(row[4:7]) for row in measured])
        outward = -rotations[:,:,2]
        nominal_angle = np.rad2deg(np.arccos(np.clip(outward[:,2],-1,1)))
        x,y,_,envelope = truth
        dy,dx = np.gradient(envelope,y[1]-y[0],x[1]-x[0])
        local = self.local(measured[:,1:4]-self.radius*rotations[:,:,2])
        coords = np.vstack([(local[:,1]-y[0])/(y[1]-y[0]),(local[:,0]-x[0])/(x[1]-x[0])])
        normal = np.c_[-map_coordinates(dx,coords,order=1,mode='nearest'),
                       -map_coordinates(dy,coords,order=1,mode='nearest'),np.ones(len(local))]
        normal /= np.linalg.norm(normal,axis=1,keepdims=True)
        normal = normal @ self.pose[:3,:3].T
        surface_angle = np.rad2deg(np.arccos(np.clip(np.einsum('ij,ij->i',normal,outward),-1,1)))
        data = np.c_[records[:,:5],np.zeros(len(records)),nominal_angle,surface_angle,records[:,8]]
        filtered = self.gaussian.apply(records[:, 0], records[:, 4], records[:, 8])
        filtered_positions = self.gaussian.apply(records[:, 0], records[:, 1:4], records[:, 8])
        speed_samples = []
        for path in np.unique(records[:,8]):
            at = np.flatnonzero(records[:,8]==path)
            # A 0.2 s displacement window suppresses quantisation in service-polled TCP positions.
            for k in at:
                first = at[np.searchsorted(records[at,0],records[k,0]-.2)]
                dt = records[k,0]-records[first,0]
                if dt>.05:
                    data[k,5] = np.linalg.norm(records[k,1:3]-records[first,1:3])/dt
                    speed_samples.append(data[k,5])
        data[:,0] -= data[0,0]
        columns = ('t_s,x_mm,y_mm,z_mm,force_n,speed_mm_s,nominal_angle_deg,observable_normal_angle_deg,path,'
                   'filtered_force_n,target_force_n')
        np.savetxt(root/'samples.csv',np.c_[data,filtered,np.full(len(data),self.force_target)],delimiter=',',header=columns,comments='')
        np.savetxt(root/'pose_wrench.csv',measured,delimiter=',',
                   header='t_s,x_mm,y_mm,z_mm,a_deg,b_deg,c_deg,Fx_N,Fy_N,Fz_N,Tx_Nm,Ty_Nm,Tz_Nm',comments='')
        if self.plot_force:
            fig,axes = self.subplots(3,1,figsize=(12,9),sharex=True)
            for path in np.unique(records[:,8]):
                at = records[:,8]==path
                axes[0].plot(data[at,0],data[at,4],alpha=.35,color='C0')
                axes[0].plot(data[at,0],filtered[at],color='C1')
                axes[1].plot(data[at,0],data[at,5],color='C0')
                axes[2].plot(data[at,0],data[at,6],color='C0',label='nominal' if path==records[0,8] else None)
                axes[2].plot(data[at,0],data[at,7],color='C1',alpha=.7,label='observable normal' if path==records[0,8] else None)
            axes[0].axhline(self.force_target,color='r',ls='--',label='target reaction')
            axes[0].set(title=f'Raw reaction (blue) / Gaussian sigma={self.gaussian.sigma*1000:g} ms (orange)',ylabel='Reaction [N]'); axes[0].legend()
            axes[1].axhline(self.feed_target,color='r',ls='--'); axes[1].set_ylabel('Feed [mm/s]')
            axes[2].set(xlabel='Time [s]',ylabel='Angle error [deg]'); axes[2].legend()
            self.close_figure(fig, root/'control.png')
        fig,axis = self.subplots(figsize=(7,6))
        for path in np.unique(records[:,8]):
            at = records[:,8]==path
            axis.plot(data[at,1],data[at,2],lw=.5,alpha=.5,color='steelblue',label='Raw' if path==records[0,8] else None)
            axis.plot(filtered_positions[at,0],filtered_positions[at,1],lw=.6,color='darkorange',label='Gaussian' if path==records[0,8] else None)
        axis.set(xlabel='BASE X [mm]',ylabel='BASE Y [mm]',aspect='equal',title='Measured TCP paths')
        axis.legend(); self.close_figure(fig, root/'trajectory.png')
        contact = data[:,4]>self.threshold
        return dict(contact_retention_percent=float(100*contact.mean()),
                    force_rmse_n=float(np.sqrt(np.mean((data[:,4]-self.force_target)**2))),
                    gaussian_force_rmse_n=float(np.sqrt(np.mean((filtered-self.force_target)**2))),
                    raw_peak_force_n=float(np.max(data[:, 4])), gaussian_peak_force_n=float(np.max(filtered)),
                    feed_median_mm_s=float(np.median(speed_samples)) if speed_samples else None,
                    nominal_angle_p95_deg=float(np.percentile(nominal_angle,95)),
                    observable_normal_angle_p95_deg=float(np.percentile(surface_angle,95)))
