"""Return to configured HOME through the existing clearance route."""
import numpy as np

if __package__:
    from repair_car.robot_control.rotations import rotation_zyz
else:
    from robot_control.rotations import rotation_zyz


class HomeMotion:
    JOINT_TOLERANCE_DEG = 1.

    def __init__(self, robot, probe, config, travel_z):
        self.robot, self.probe, self.config, self.travel_z = robot, probe, config, travel_z

    def run(self, joints=None, *, scan_transfer=None):
        joints = self.config['robot']['home_joints_deg'] if joints is None else joints
        self.robot.check_cancelled()
        self.probe.close()
        current = self.robot.get_current_posj()
        motion = self.config['motion']
        pose, _ = self.probe.read()
        target = np.asarray(self.robot.fkin(joints), float)
        target[:3] += rotation_zyz(target[3:]) @ self.probe.offset
        rotation = rotation_zyz(pose[3:])
        if (np.max(np.abs(np.subtract(current, joints))) <= self.JOINT_TOLERANCE_DEG
                and np.linalg.norm(pose[:3] - target[:3]) < 1.
                and self.probe.turn_deg(pose[3:], rotation_zyz(target[3:])) < 1.):
            return
        if pose[2] <= self.travel_z + 50. and np.linalg.norm(pose[:2] - target[:2]) > 5.:
            z = max(pose[2], self.travel_z)
            if scan_transfer is not None:
                scan_transfer(pose, target[:2], z, rotation)
            else:
                self.probe.move([*pose[:2], z], rotation, motion['approach_speed'], acc=motion['approach_acc'])
                self.probe.move([*target[:2], z], rotation, motion['linear_speed'], acc=motion['linear_acc'])
        self.robot.movej(joints, motion['joint_speed'], motion['joint_acc'])
