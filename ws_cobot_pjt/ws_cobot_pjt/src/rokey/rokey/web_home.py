"""Return HOME through the same clearance path used by the scan programs (virtual session or real robot)."""
from .program import RobotProgram


def main():
    program = RobotProgram('web_home')
    try:
        program.home()
    finally:
        program.close()


if __name__ == '__main__':
    main()
