"""URDF BASE-to-flange FK/Jacobian and local IK, including the active ProbeTCP (ZYZ profile)."""
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

from repair_car.robot_control.rotations import axis_angle, rotation_zyz


class RobotChain:
    def __init__(self, description, tcp_profile, settings):
        self.description = description
        robot = ET.fromstring(description)
        by_child = {joint.find('child').get('link'): joint for joint in robot.findall('joint')}
        chain, child = [], 'link_6'
        while child != 'base_link':
            joint = by_child[child]
            chain.insert(0, joint)
            child = joint.find('parent').get('link')
        self.joints, self.names, limits = [], [], []
        for joint in chain:
            element = joint.find('origin')
            xyz = np.fromstring(element.get('xyz', '0 0 0'), sep=' ') if element is not None else np.zeros(3)
            rpy = np.fromstring(element.get('rpy', '0 0 0'), sep=' ') if element is not None else np.zeros(3)
            fixed = np.eye(4)
            fixed[:3, :3], fixed[:3, 3] = Rotation.from_euler('xyz', rpy).as_matrix(), xyz
            axis = joint.find('axis')
            direction = np.fromstring(axis.get('xyz', '0 0 1'), sep=' ') if axis is not None else np.array([0., 0., 1.])
            active = joint.get('type') in ('revolute', 'continuous')
            self.joints.append((fixed, direction, active))
            if active:
                self.names.append(joint.get('name'))
                limit = joint.find('limit')
                limits.append([float(limit.get('lower', '-3.14159')), float(limit.get('upper', '3.14159'))])
        if len(self.names) != 6:
            raise ValueError('Expected the six-joint M0609 BASE-to-flange chain')
        self.tcp = np.eye(4)
        self.tcp[:3, 3] = np.asarray(tcp_profile[:3]) / 1000
        self.tcp[:3, :3] = rotation_zyz(tcp_profile[3:])
        margin = np.deg2rad(settings['joint_margin_deg'])
        self.lower, self.upper = np.asarray(limits)[:, 0] + margin, np.asarray(limits)[:, 1] - margin
        self.settings = settings
        pose, _, joints = self.forward(np.zeros(6))
        shoulder, elbow, wrist = joints[1:4]
        self.shoulder = shoulder
        self.arm_length = np.linalg.norm(elbow - shoulder) + np.linalg.norm(wrist - elbow)
        self.tcp_to_wrist = pose[:3, :3].T @ (wrist - pose[:3, 3])

    def within_outer_reach(self, positions, rotation, margin):
        """M0609 outer reach for TCP positions in metres at a fixed rotation."""
        wrist = positions + rotation @ self.tcp_to_wrist
        # The shoulder's lateral offset rotates with joint 1 and is perpendicular to the arm's reach plane.
        radial_sq = np.sum(wrist[:, :2] ** 2, axis=1) - self.shoulder[:2] @ self.shoulder[:2]
        distance_sq = np.maximum(radial_sq, 0.) + (wrist[:, 2] - self.shoulder[2]) ** 2
        return distance_sq <= (self.arm_length - margin) ** 2

    def forward(self, q):
        transform, origins, axes, index = np.eye(4), [], [], 0
        for fixed, axis, active in self.joints:
            transform = transform @ fixed
            if active:
                origins.append(transform[:3, 3].copy())
                axes.append(transform[:3, :3] @ axis)
                motion = np.eye(4)
                motion[:3, :3] = axis_angle(axis, q[index])
                transform = transform @ motion
                index += 1
        flange = transform[:3, 3].copy()
        pose = transform @ self.tcp
        jacobian = np.column_stack([np.r_[np.cross(axis, pose[:3, 3] - origin), axis]
                                   for origin, axis in zip(origins, axes)])
        return pose, jacobian, np.asarray([*origins, flange])

    def solve(self, position, rotation, seed, *, transfer=False):
        q = np.clip(np.asarray(seed).copy(), self.lower, self.upper)
        c = self.settings
        tolerance = c['ik_position_mm']
        minimum = c['minimum_singular_value']
        if transfer:
            tolerance = min(tolerance, c.get('transfer_position_mm', .1))
            minimum = max(minimum, c.get('transfer_minimum_singular_value', .005))
        for _ in range(80):
            pose, jacobian, _ = self.forward(q)
            dp = position - pose[:3, 3]
            dr = Rotation.from_matrix(rotation @ pose[:3, :3].T).as_rotvec()
            weighted = jacobian * np.array([1., 1., 1., .15, .15, .15])[:, None]
            if np.linalg.norm(dp) < tolerance / 1000 and np.linalg.norm(dr) < np.deg2rad(c['ik_angle_deg']):
                return q if np.linalg.svd(weighted, compute_uv=False)[-1] >= minimum else None
            error = np.r_[dp, .15 * dr]
            step = weighted.T @ np.linalg.solve(weighted @ weighted.T + .0001 * np.eye(6), error)
            step *= min(1., np.deg2rad(c['joint_step_deg']) / max(np.max(np.abs(step)), 1e-12))
            q = np.clip(q + step, self.lower, self.upper)
        return None
