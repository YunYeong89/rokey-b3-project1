"""Web viewer, rosbridge and web_control started together with a native session (--web; virtual or real robot).
Without --web, ./run_web.sh start attaches the same web to a session that is already running.
The viewer shows the robot (joint states of `joint_topic`), the --car scene of a simulation if any (markers on
/dsr01/sim_scene, meshes served from the session's cache by robot_files.py), the programs' clouds and report files."""
from pathlib import Path
import os
import shlex
import sys

from robot_model import RobotModelAssets


class SimulationWeb:
    def __init__(self, directory, port=5050, joint_topic='/dsr01/sim_joint_states'):
        self.directory, self.port, self.joint_topic = Path(directory), port, joint_topic

    def export_vehicles(self, root):
        """static/models/<type>.glb for the 3D car viewer, from the rokey vehicle models at their current scale."""
        sys.path.insert(0, str(root/'ws_cobot_pjt/src/repair_car'))
        from repair_car.config import load
        from repair_car.geometry.vehicle_model import VehicleModel
        models = self.directory/'static/models'
        models.mkdir(parents=True, exist_ok=True)
        for kind in load('vehicles')['types']:
            VehicleModel(kind).export_glb(models/f'{kind}.glb')

    @staticmethod
    def environment(root, port):
        """Flask settings shared with run_web.sh: result folders and scene meshes of the workspace."""
        return {'PORT': str(port), 'ROKEY_DIR': str(root/'ws_cobot_pjt/src/repair_car'),
                'SESSION_DIR': str(root/'ws_cobot_pjt/src/repair_car/data'),
                'SCENE_DIRS': os.pathsep.join([str(root/'ws_cobot_pjt/src/repair_car/native/.cache/scene'),
                                               str(root/'ws_cobot_pjt/src/repair_car/meshes')])}

    def start(self, session, model):
        python = self.directory/'.venv/bin/python'
        if not python.is_file():
            raise RuntimeError(f'Install web dependencies first: cd {self.directory} && '
                               'python3 -m venv .venv && .venv/bin/pip install -r requirements.txt')
        from model_export import ROOT
        root = ROOT
        RobotModelAssets(self.directory).export(model.urdf, root/'ws_cobot_pjt/src',
                                                getattr(model, 'tcp_link', 'tool0'))
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
            f'exec env {environment} python3 -u -m repair_car.web.web_control', 'web_control')
        session.start([str(python), '-u', str(self.directory/'app.py')],
                      env={**self.environment(root, self.port), 'ROBOT_JOINT_TOPIC': self.joint_topic})
        print(f'Web robot control: http://localhost:{self.port}/admin?tab=robot '
              '(login, then open the robot tab)', flush=True)
