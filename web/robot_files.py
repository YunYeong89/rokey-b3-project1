"""Robot program outputs for the web robot tab, read from the shared workspace (admin only):
  GET /api/robot/runs                  report files grouped by run (<name>_YYYYMMDD_HHMMSS), newest first: rokey/images
                                       and rokey/pointcloud, also one level inside a run directory
  GET /api/robot/files/<folder>/<path> one file of images or pointcloud (images/force_live.png: the live force graph)
  GET /api/robot/clouds/<name>         the running program's RViz cloud, session/clouds/<name>.npy (float32 BASE m)
  GET /api/robot/scene/<name>.stl      a mesh of the simulation scene (/dsr01/sim_scene markers reference
                                       scene/<name>.stl), found by name in the scene directories, first match"""
from pathlib import Path
import re

from flask import abort, jsonify, request, send_file, send_from_directory

RUN = re.compile(r'^(.+?_\d{8}_\d{6})')
KINDS = {'.png': 'image', '.csv': 'data', '.json': 'data', '.ply': 'cloud'}
CLOUDS = ('reference', 'measured')


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
        return {name: root / name for name in ('images', 'pointcloud')}

    @staticmethod
    def files(folders):
        """(folder, run, path) of every report file."""
        for folder, root in folders.items():
            for top in (root.iterdir() if root.is_dir() else []):
                run = RUN.match(top.name)
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
        for root in self.scene_dirs:
            match = next((path for path in root.rglob('*.stl') if path.name == name), None) if root.is_dir() else None
            if match:
                return send_file(match, max_age=0)
        abort(404)

    def register(self, app, guard):
        app.add_url_rule('/api/robot/scene/<name>', 'robot_scene', guard(self.scene))
        app.add_url_rule('/api/robot/runs', 'robot_runs', guard(lambda: jsonify(self.runs())))
        app.add_url_rule('/api/robot/files/<folder>/<path:name>', 'robot_file', guard(self.file))
        app.add_url_rule('/api/robot/clouds/<name>', 'robot_cloud', guard(self.cloud))
