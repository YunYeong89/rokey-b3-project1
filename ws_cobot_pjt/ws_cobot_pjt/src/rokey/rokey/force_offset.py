"""Force offset of the held probe, away from the table and the vehicle: from HOME straight up to
force_offset.height_mm (BASE z), the tool down as in the scan, still; the controller's BASE wrench estimate averaged
over force_offset.seconds in position control, then under the scan's compliance with a 0 N desired force along z.
-> session/force_offset.json. real_scan (real robot) tares its readings with the force-control value and adds it to
the commanded force, so that the pressing force is the one set.
  ros2 run rokey force_offset      (the probe picked, HOME clear)"""
import json
import time

import numpy as np

from .configs import read_session, write_session
from .program import RobotProgram
from .rotations import rotation_zyz


class ForceOffset:
    def __init__(self, program):
        self.program, self.robot, self.probe = program, program.robot, program.probe
        self.settings = program.config['force_offset']

    def average(self):
        """Mean and spread of the untared BASE wrench over `seconds`, the robot still."""
        time.sleep(self.settings['settle_s'])
        values, end = [], time.monotonic() + self.settings['seconds']
        while time.monotonic() < end:
            values.append(self.robot.get_tool_force())
        values = np.asarray(values)
        return dict(mean=values.mean(0).round(4).tolist(), std=values.std(0).round(4).tolist(), samples=len(values))

    def run(self):
        p, m = self.program, self.program.config['motion']
        p.home()
        pose, _ = self.probe.read()
        rotation = rotation_zyz(pose[3:])
        self.probe.move([*pose[:2], max(pose[2], self.settings['height_mm'])], rotation, m['approach_speed'],
                        acc=m['approach_acc'])
        pose, _ = self.probe.read()
        position = self.average()
        self.probe.enable(self.robot.DR_BASE, p.config['real_scan']['stiffness'])
        self.robot.set_desired_force([0.] * 6, [0, 0, 1, 0, 0, 0], mod=self.robot.DR_FC_MOD_ABS)
        self.probe.forcing = True
        try:
            force = self.average()
        finally:
            self.probe.close()
        p.home()
        return write_session('force_offset.json', dict(
            mode=p.mode, tool=self.robot.get_tool(), tcp=self.robot.get_tcp(),
            pose_mm_deg=np.round(pose, 2).tolist(), position_control=position, force_control=force))

    @staticmethod
    def load(program):
        """The stored force-control offset (BASE wrench) for this mode and active tool, else None."""
        data = read_session('force_offset.json')
        if data and data['mode'] == program.mode and data['tool'] == program.robot.get_tool():
            return np.asarray(data['force_control']['mean'])
        return None


def main():
    program = RobotProgram('force_offset')
    try:
        path = ForceOffset(program).run()
        data = json.loads(path.read_text())
        for state in ('position_control', 'force_control'):
            program.log(f'{state}: F {data[state]["mean"][:3]} N (std {data[state]["std"][:3]}), '
                        f'M {data[state]["mean"][3:]} N m, {data[state]["samples"]} samples')
        program.log(f'-> {path} (tool {data["tool"]})')
    finally:
        program.close()


if __name__ == '__main__':
    main()
