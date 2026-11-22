"""Ordinary approach/retract moves retain the existing DSR motion speeds and interface."""
import numpy as np

from ..rotations import rotation_zyz


class ScanTransfers:
    def __init__(self, program, scan, down, outward=None):
        self.program, self.probe, self.scan, self.down = program, program.probe, scan, down
        self.motion = program.config['motion']
        self.outward = outward

    def normal(self, rotation):
        return -rotation[:, 2] if self.outward is None else self.outward

    def approach(self, trajectory):
        first, rotation, _ = trajectory.sample(0.)
        normal = self.normal(rotation)
        out = first + self.scan['standoff_mm'] * normal
        pose, _ = self.probe.read()
        travel = max(self.program.safe_z(), out[2], pose[2])
        m = self.motion
        self.probe.move([*pose[:2], travel], rotation_zyz(pose[3:]), m['approach_speed'], acc=m['approach_acc'])
        self.probe.move([*out[:2], travel], self.down, m['linear_speed'], acc=m['linear_acc'])
        self.probe.move(out, self.down, m['approach_speed'], acc=m['approach_acc'])
        self.probe.move(out, rotation, m['approach_speed'], acc=m['approach_acc'])
        self.probe.move(first + self.scan['gap_mm'] * normal, rotation,
                        m['approach_speed'], acc=m['approach_acc'])
        self.probe.move(first, rotation, self.scan['mesh_normal']['approach_speed_mm_s'], acc=m['approach_acc'])

    def retract(self):
        pose, _ = self.probe.read()
        rotation = rotation_zyz(pose[3:])
        normal, m = self.normal(rotation), self.motion
        self.probe.move(pose[:3] + self.scan['gap_mm'] * normal, rotation,
                        self.scan['mesh_normal']['approach_speed_mm_s'], acc=m['approach_acc'])
        out = pose[:3] + self.scan['standoff_mm'] * normal
        self.probe.move(out, rotation, m['approach_speed'], acc=m['approach_acc'])
        self.probe.move(out, self.down, m['approach_speed'], acc=m['approach_acc'])
        self.probe.move([*out[:2], max(self.program.safe_z(), out[2])], self.down,
                        m['approach_speed'], acc=m['approach_acc'])

    def resume(self, trajectory, elapsed, lift):
        """After an interrupted stroke: lift mm away from the surface, then return to the stroke pose at
        `elapsed` (trajectory time, s), where detection starts again."""
        pose, _ = self.probe.read()
        rotation = rotation_zyz(pose[3:])
        speed, acc = self.scan['mesh_normal']['approach_speed_mm_s'], self.motion['approach_acc']
        self.probe.move(pose[:3] + lift * self.normal(rotation), rotation, speed, acc=acc)
        target, rotation, _ = trajectory.sample(elapsed)
        self.probe.move(target + lift * self.normal(rotation), rotation, speed, acc=acc)
        self.probe.move(target, rotation, speed, acc=acc)
