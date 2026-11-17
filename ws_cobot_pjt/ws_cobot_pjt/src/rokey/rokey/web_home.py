"""Return HOME through the same clearance path used by the scan programs."""
from .program import RobotProgram


def main():
    program = RobotProgram('web_home')
    try:
        if program.mode != 'virtual':
            raise RuntimeError('Web HOME requires the native virtual session')
        program.home()
    finally:
        program.close()


if __name__ == '__main__':
    main()
