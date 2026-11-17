"""Part replacement - skeleton. The interface the process uses is final:
  PartReplacer(program).replace(part) -> {'success', 'retries', 'peak_force', 'flush_mm'}
The motion is not implemented yet (tool change probe <-> part gripper, part grasp shapes, new-part tray and
scrap positions, force-controlled mounting, flush re-inspection). In the simulation every step is only
logged, the result is a success and the reference cloud marks the part as replaced; the real robot refuses.
  ros2 run rokey replace_parts --parts bonnet
"""
import argparse

if __package__:
    from .program import RobotProgram
else:
    from program import RobotProgram


class PartReplacer:
    steps = ['tool change: probe -> part gripper', 'remove the damaged part', 'drop it at the scrap position',
             'pick a new part from the tray', 'mount it under TOOL Z force control', 'tool change: part gripper -> probe',
             'flush re-inspection along the part outline']

    def __init__(self, program):
        self.program = program

    def replace(self, part):
        if self.program.mode != 'virtual':
            raise NotImplementedError('실기 부품 교체 동작은 아직 구현되지 않았습니다 (replace_parts 뼈대)')
        for index, step in enumerate(self.steps):
            self.program.log(f'{part} {index + 1}/{len(self.steps)}: {step} (simulated, no motion)')
        self.program.publish(type='replaced', part=part)
        return {'success': True, 'retries': 0, 'peak_force': None, 'flush_mm': None}


def run(parts):
    program = RobotProgram('replace_parts')
    try:
        replacer = PartReplacer(program)
        for part in parts:
            program.log(f'{part}: {replacer.replace(part)}')
    finally:
        program.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--parts', nargs='+', required=True, help='part codes, e.g. bonnet mirror_r')
    run(parser.parse_args().parts)


if __name__ == '__main__':
    main()
