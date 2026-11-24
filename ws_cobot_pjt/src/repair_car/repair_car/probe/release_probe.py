"""Place the probe in the selected robot's rack and return HOME."""
import argparse

from repair_car.config import load, read_session
from repair_car.probe.pick_probe import ProbePickup
from repair_car.robot_control.program import RobotProgram
from repair_car.probe.real_probe_pickup import RealProbePickup
from repair_car.robot_control.robot_bridge import program_args


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.parse_args(program_args())
    program = RobotProgram('release_probe')
    try:
        if program.mode == 'real':
            RealProbePickup(program.robot, load('real_scene')['probe'], program.config['robot']['home_joints_deg']).run(release=True)
        else:
            plan = read_session('probe_plan.json')
            if not plan or 'dock_probe_tcp_mm_deg' not in plan:
                parser.error('No virtual probe docking plan. Restart the session with the --car option.')
            ProbePickup(program.node, program.robot, plan, program.config['motion']).dock()
    finally:
        program.close()


if __name__ == '__main__':
    main()
