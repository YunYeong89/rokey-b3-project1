"""Command to run the namespaced ROS stack inside the shared container."""
import shlex


BRINGUP = '/workspace/src/cobot_rg2/rg2/m0609_rg2_bringup'
ROS_SETUP = (
    'source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && '
    f'export PYTHONPATH=/workspace/src/rokey:{BRINGUP}:"${{PYTHONPATH:-}}" && '
)


def launch_command(session, network_env, bridge_port):
    virtual = session.args.mode == 'virtual'
    host = f'emulator_{session.robot_id}' if virtual else session.args.host
    arguments = {
        'robot_ns': session.robot_id,
        'mode': session.args.mode,
        'host': host,
        'rt_host': session.args.rt_host or host,
        'virtual_force': str(virtual).lower(),
        'virtual_profile_file': session.calibration_path or '/workspace/src/rokey/calibration.json',
        'bridge_port': bridge_port,
        'bridge_token': session.token,
    }
    command = ['env', *(f'{key}={value}' for key, value in network_env.items()),
               'ros2', 'launch', f'{BRINGUP}/launch/service.launch.py',
               *(f'{key}:={value}' for key, value in arguments.items())]
    return ROS_SETUP + 'exec ' + shlex.join(command)
