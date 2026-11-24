"""Robot program outputs for the web robot tab, read from the shared workspace (admin only):
  GET /api/robot/runs                  report files grouped by run (<name>_YYYYMMDD_HHMMSS), newest first: rokey/images
                                       and rokey/pointcloud, also one level inside a run directory
  GET /api/robot/files/<folder>/<path> one file of images or pointcloud (images/force_live.png: the live force graph)
  GET /api/robot/clouds/<name>         the running program's RViz cloud, session/clouds/<name>.npy (float32 BASE m)
  GET /api/robot/scene/<name>.stl      a mesh of the simulation scene (/dsr01/sim_scene markers reference
                                       scene/<name>.stl), found by name in the scene directories, first match
  vehicle_cloud(started)              the running scan's measured points in the vehicle frame for the owner's car view
                                       (app.py /api/jobs/<id>/cloud): mm, glTF axes of car.glb (x front from the rear, y up,
                                       z right from the left side) at the real vehicle's size; the page scales to its model
  GET /api/robot/fixed-scene           the real cell without a simulation: the team vehicle at its measured pose
                                       (rokey config/real_scene.yaml vehicle), as /sim_scene markers (meshes
                                       static/robot/scene/real_<part>.stl, made by rokey RealVehicle().export)"""
from pathlib import Path
import json
import math
import re

from flask import abort, jsonify, request, send_file, send_from_directory

RUN = re.compile(r'^(.+?_\d{8}_\d{6})')
INSPECTION_DATE = re.compile(r'\d{4}(?:-\d{2}){4}')
KINDS = {'.png': 'image', '.csv': 'data', '.json': 'data', '.ply': 'cloud'}
CLOUDS = ('reference', 'measured')
VEHICLE_PARTS = ('body', 'door', 'bonnet', 'bumper', 'other')   # real_vehicle.py part_colors


class RobotFiles:
    def __init__(self, rokey_dir, session_dir, scene_dirs=()):
        """scene_dirs: the running session's mesh cache (native/.cache/scene) first, then the package meshes."""
        self.rokey, self.session = Path(rokey_dir), Path(session_dir)
        self.scene_dirs = [Path(directory) for directory in scene_dirs]

    def roots(self):
        robot = request.args.get('robot', 'dsr01')
        if robot == 'dsr01':
            return self.rokey, self.session
        if not re.fullmatch(r'robot_[1-9][0-9]*', robot):
            abort(404)
        return self.session / robot, self.session / robot

    def folders(self):
        root, _ = self.roots()
        return {name: root / name for name in ('data', 'images', 'pointcloud')}

    @staticmethod
    def files(folders):
        """(folder, run, path) of every report file."""
        for folder, root in folders.items():
            if folder == 'data':
                for part in root.glob('*/*/*'):
                    if part.is_dir() and INSPECTION_DATE.fullmatch(part.parent.name):
                        for path in part.rglob('*'):
                            if path.suffix in KINDS and path.is_file():
                                yield folder, part.relative_to(root).as_posix(), path
            for top in (root.iterdir() if root.is_dir() else []):
                run = RUN.match(top.name)
                if not run:
                    continue
                for path in (sorted(top.iterdir()) if top.is_dir() else [top]):
                    if run and path.suffix in KINDS and path.is_file():
                        yield folder, run[1], path

    def runs(self, limit=20):
        runs = {}
        folders = self.folders()
        for folder, run, path in self.files(folders):
            stat = path.stat()
            entry = runs.setdefault(run, dict(run=run, mtime=0., files=[]))
            entry['mtime'] = max(entry['mtime'], stat.st_mtime)
            entry['files'].append(dict(path=f'{folder}/{path.relative_to(folders[folder]).as_posix()}',
                                       name=path.relative_to(folders[folder]).as_posix(),
                                       kind=KINDS[path.suffix], size=stat.st_size, mtime=stat.st_mtime))
        newest = sorted(runs.values(), key=lambda entry: -entry['mtime'])[:limit]
        for entry in newest:
            entry['files'].sort(key=lambda file: (file['kind'] != 'image', file['name']))
        return newest

    def file(self, folder, name):
        folders = self.folders()
        if folder not in folders:
            abort(404)
        return send_from_directory(folders[folder], name, max_age=0)

    def cloud(self, name):
        if name not in CLOUDS:
            abort(404)
        _, session = self.roots()
        return send_from_directory(session / 'clouds', f'{name}.npy', max_age=0, mimetype='application/octet-stream')

    def scene(self, name):
        robot = request.args.get('robot', 'dsr01')
        self.roots()
        roots = self.scene_dirs
        if robot != 'dsr01' and roots:
            roots = [roots[0].parent / 'service' / robot / 'scene', *roots[1:]]
        for root in roots:
            # Shallowest first: meshes/probes/<name> (tip frame) before meshes/probes/original/<name>.
            match = min(root.rglob(name), key=lambda path: len(path.parts), default=None) if root.is_dir() else None
            if match:
                return send_file(match, max_age=0)
        abort(404)

    def real_placement(self, session):
        import yaml
        real = yaml.safe_load((self.rokey / 'config/real_scene.yaml').read_text())
        vehicle = real['vehicle']
        spec = yaml.safe_load((self.rokey / 'config/vehicles.yaml').read_text())['real']
        for name in ('inspection_scene.json', 'real_door_scene.json'):
            path = session / name
            if path.is_file():
                placement = json.loads(path.read_text())
                return real, placement['pose_base_mm_deg'], placement['base_z_mm']
        z = vehicle['base_z_mm'] if vehicle.get('base_z_mm') is not None else vehicle.get('table_z_mm', 0.) + spec.get('lift_mm', 0.)
        return real, vehicle['pose_base_mm_deg'], z

    def fixed_scene(self):
        """The team vehicle where it always stands (real_scene.yaml), in the MarkerArray JSON of /sim_scene."""
        import yaml
        try:
            _, session = self.roots()
            real, (x, y, yaw), z = self.real_placement(session)
            spec = yaml.safe_load((self.rokey / 'config/vehicles.yaml').read_text())['real']
        except (OSError, KeyError, TypeError) as error:
            return jsonify(error=f'실차 배치 설정을 읽을 수 없습니다: {error}'), 404
        body = [float(v) for v in spec.get('color', (0.36, 0.42, 0.30))]
        colors = {'body': body, 'door': [min(v * 1.25, 1) for v in body], 'bonnet': [min(v * 1.5, 1) for v in body],
                  'bumper': [0.16, 0.16, 0.17], 'other': [0.07, 0.07, 0.08]}
        half = math.radians(yaw) / 2
        markers = [dict(ns='fixed_vehicle', id=index, type=10, mesh_resource=f'scene/real_{part}.stl',
                        pose=dict(position=dict(x=x / 1000, y=y / 1000, z=z / 1000),
                                  orientation=dict(x=0.0, y=0.0, z=math.sin(half), w=math.cos(half))),
                        scale=dict(x=0.001, y=0.001, z=0.001),
                        color=dict(r=colors[part][0], g=colors[part][1], b=colors[part][2], a=1.0))
                   for index, part in enumerate(VEHICLE_PARTS)]
        return jsonify(markers=markers, pose_base_mm_deg=[x, y, yaw], base_z_mm=z,
                       probe_tcp_mm_deg=(real.get('probe') or {}).get('tcp_mm_deg'))   # the held probe in the viewer

    MAX_POINTS = 6000   # owner's view: enough to see the strokes, small enough to poll every 2 s

    def vehicle_frame(self):
        """(pose x, y, yaw [mm, deg], base z, bounds min/max [mm]) of the real vehicle: real_scene.yaml and the
        vehicle-frame meshes static/robot/scene/real_*.stl (x front, y left, z up), cached by file times."""
        files = [self.rokey / 'config/real_scene.yaml', self.rokey / 'config/vehicles.yaml',
                 *sorted(p for p in self.scene_dirs[-1].glob('real_*.stl'))]
        stamp = tuple(f.stat().st_mtime for f in files if f.exists())
        if getattr(self, '_frame', (None,))[0] != stamp:
            import numpy as np
            import yaml
            cell = yaml.safe_load(files[0].read_text())['vehicle']
            spec = yaml.safe_load(files[1].read_text())['real']
            z = cell['base_z_mm'] if cell.get('base_z_mm') is not None else cell.get('table_z_mm', 0.0) + spec.get('lift_mm', 0.0)
            low, high = np.full(3, np.inf), np.full(3, -np.inf)
            for path in files[2:]:
                data = path.read_bytes()
                vertices = np.frombuffer(data, dtype=[('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')],
                                         count=int.from_bytes(data[80:84], 'little'), offset=84)['v'].reshape(-1, 3)
                low, high = np.minimum(low, vertices.min(0)), np.maximum(high, vertices.max(0))
            self._frame = stamp, (*cell['pose_base_mm_deg'], z, low, high)
        return self._frame[1]

    def vehicle_cloud(self, started, robot='dsr01'):
        """Measured contact points of the scan running since `started` (epoch s), or None: session/clouds/measured.npy
        (BASE m) -> vehicle frame -> car.glb axes [mm]. Returns (flat float list, real size [L, H, W] mm, file time)."""
        import numpy as np
        if robot != 'dsr01' and not re.fullmatch(r'robot_[1-9][0-9]*', robot):
            abort(404)
        session = self.session if robot == 'dsr01' else self.session / robot
        path = session / 'clouds' / 'measured.npy'
        try:
            stat = path.stat()
        except FileNotFoundError:
            return None
        if stat.st_mtime < started:   # a previous job's points (each scan clears the file when it starts)
            return None
        points = np.load(path).astype(float)[:, :3] * 1000.
        _, _, _, _, low, high = self.vehicle_frame()
        _, (x, y, yaw), z = self.real_placement(session)
        c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
        d = points - [x, y, z]
        local = np.c_[c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1], d[:, 2]]   # Rz(-yaw)
        if len(local) > self.MAX_POINTS:
            local = local[np.linspace(0, len(local) - 1, self.MAX_POINTS).astype(int)]
        gltf = np.c_[local[:, 0] - low[0], local[:, 2] - low[2], high[1] - local[:, 1]]
        size = [float(high[0] - low[0]), float(high[2] - low[2]), float(high[1] - low[1])]
        return np.round(gltf, 1).ravel().tolist(), size, stat.st_mtime

    def register(self, app, guard):
        app.add_url_rule('/api/robot/scene/<name>', 'robot_scene', guard(self.scene))
        app.add_url_rule('/api/robot/fixed-scene', 'robot_fixed_scene', guard(self.fixed_scene))
        app.add_url_rule('/api/robot/runs', 'robot_runs', guard(lambda: jsonify(self.runs())))
        app.add_url_rule('/api/robot/files/<folder>/<path:name>', 'robot_file', guard(self.file))
        app.add_url_rule('/api/robot/clouds/<name>', 'robot_cloud', guard(self.cloud))
