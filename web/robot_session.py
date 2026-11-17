"""Attach the web viewer and rosbridge to an existing native simulation session.
The viewer shows the same scene as the simulation: the robot, plus the --car scene if any
(markers on /dsr01/sim_scene, meshes exported to static/robot/scene)."""
from pathlib import Path
import os
import shlex
import shutil
import sys
from urllib.parse import unquote, urlparse

from robot_model import RobotModelAssets


class SimulationWeb:
    def __init__(self, directory, port=5050):
        self.directory, self.port = Path(directory), port

    def export_vehicles(self, root):
        """static/models/<type>.glb for the 3D car viewer, from the rokey vehicle models at their current scale."""
        sys.path.insert(0, str(root/'ws_cobot_pjt/src/rokey/rokey'))
        from configs import load
        from vehicle_model import VehicleModel
        models = self.directory/'static/models'
        models.mkdir(parents=True, exist_ok=True)
        for kind in load('vehicles')['types']:
            VehicleModel(kind).export_glb(models/f'{kind}.glb')

    def start(self, session, model):
        python = self.directory/'.venv/bin/python'
        if not python.is_file():
            raise RuntimeError(f'Install web dependencies first: cd {self.directory} && '
                               'python3 -m venv .venv && .venv/bin/pip install -r requirements.txt')
        root = model.urdf.parents[2]
        RobotModelAssets(self.directory).export(model.urdf, root/'ws_cobot_pjt/src',
                                                getattr(model, 'tcp_link', 'tool0'))
        scene, scene_dir = getattr(model, 'scene_map', None), self.directory/'static/robot/scene'
        shutil.rmtree(scene_dir, ignore_errors=True)
        if scene is not None:
            scene_dir.mkdir(parents=True)
            for marker in scene.markers().markers:  # every mesh the scene references, as STL
                if marker.mesh_resource:
                    source = Path(unquote(urlparse(marker.mesh_resource).path)).with_suffix('.stl')
                    shutil.copy2(source, scene_dir/source.name)
        self.export_vehicles(root)
        session.start_remote(
            'source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && '
            'exec ros2 launch rosbridge_server rosbridge_websocket_launch.xml '
            'address:=0.0.0.0 port:=9090', 'web_bridge')
        control_env = {
            'REPAIR_DB_URL': f'http://host.docker.internal:{self.port}',
            'REPAIR_API_KEY': os.environ.get('REPAIR_API_KEY', 'rokey-robot-key'),
        }
        environment = ' '.join(f'{key}={shlex.quote(value)}' for key, value in control_env.items())
        session.start_remote(
            'source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && '
            f'exec env {environment} python3 -u -m rokey.web_control', 'web_control')
        session.start([str(python), '-u', str(self.directory/'app.py')],
                      env={'PORT': str(self.port), 'ROBOT_JOINT_TOPIC': '/dsr01/sim_joint_states'})
        print(f'Web robot control: http://localhost:{self.port}/admin?tab=robot '
              '(login, then open the robot tab)', flush=True)
