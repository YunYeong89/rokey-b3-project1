"""Place the probe in the real robot's rack and return HOME."""
import argparse

from .configs import load
from .program import RobotProgram
from .real_probe_pickup import RealProbePickup
from .robot_namespace import program_args


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.parse_args(program_args())
    program = RobotProgram('release_probe')
    try:
        if program.mode != 'real':
            parser.error('release_probe는 실기 로봇용입니다')
        pickup = RealProbePickup(program.robot, load('real_scene')['probe'], program.config['robot']['home_joints_deg'])
        pickup.run(release=True)
    finally:
        program.close()


if __name__ == '__main__':
    main()
