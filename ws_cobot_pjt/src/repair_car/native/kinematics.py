"""URDF kinematics and spatial wrench transforms in SI units."""
import numpy as np


def skew(vector):
    x, y, z = vector
    return np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])


def rotation(vector):
    angle = np.linalg.norm(vector)
    if angle < 1e-10:
        return np.eye(3) + skew(vector)
    axis = skew(vector / angle)
    return np.eye(3) + np.sin(angle) * axis + (1 - np.cos(angle)) * axis @ axis


def rotation_error(target, current):
    matrix = target @ current.T
    angle = np.arccos(np.clip((np.trace(matrix) - 1) / 2, -1, 1))
    vector = np.array([matrix[2, 1] - matrix[1, 2], matrix[0, 2] - matrix[2, 0],
                       matrix[1, 0] - matrix[0, 1]])
    return vector / 2 if angle < 1e-7 else vector * angle / (2 * np.sin(angle))


def origin(element):
    transform = np.eye(4)
    if element is not None:
        transform[:3, 3] = np.fromstring(element.get('xyz', '0 0 0'), sep=' ')
        roll, pitch, yaw = np.fromstring(element.get('rpy', '0 0 0'), sep=' ')
        transform[:3, :3] = (rotation(np.array([0, 0, yaw])) @
                             rotation(np.array([0, pitch, 0])) @
                             rotation(np.array([roll, 0, 0])))
    return transform


class KinematicChain:
    def __init__(self, robot, tcp_link='tool0'):
        self.tcp_link = tcp_link
        self.joints = []
        self.limits = {}
        self.names = [f'joint_{index}' for index in range(1, 7)]
        children = set()
        for joint in robot.findall('joint'):
            parent, child = joint.find('parent').get('link'), joint.find('child').get('link')
            axis = joint.find('axis')
            mimic = joint.find('mimic')
            limit = joint.find('limit')
            if limit is not None:
                self.limits[joint.get('name')] = (float(limit.get('lower', '-inf')),
                                                  float(limit.get('upper', 'inf')))
            self.joints.append((joint.get('name'), parent, child, origin(joint.find('origin')),
                                np.fromstring(axis.get('xyz') if axis is not None else '0 0 1', sep=' '),
                                joint.get('type'), mimic))
            children.add(child)
        self.root = next(link.get('name') for link in robot.findall('link')
                         if link.get('name') not in children)
        self.payload = []
        self.bodies = []
        self.downstream = {}
        for name, _, child, *_ in self.joints:
            links = {child}
            for _ in self.joints:
                links.update(link for _, parent, link, *_ in self.joints if parent in links)
            self.downstream[name] = links
        descendants = {tcp_link}
        for _ in self.joints:
            descendants.update(child for _, parent, child, *_ in self.joints if parent in descendants)
        for link in robot.findall('link'):
            inertial = link.find('inertial')
            if inertial is not None:
                body = (link.get('name'), float(inertial.find('mass').get('value')),
                        origin(inertial.find('origin'))[:3, 3])
                self.bodies.append(body)
                if link.get('name') in descendants:
                    self.payload.append(body)
        # Vectorised gravity torques: the per body x joint Python loop took ~540 us of a 100 Hz control step.
        self.masses = np.array([mass for _, mass, _ in self.bodies])
        self.centres = np.array([np.r_[cog, 1] for _, _, cog in self.bodies])
        self.below = np.array([[link in self.downstream[name] for link, _, _ in self.bodies] for name in self.names],
                              dtype=float)
        # One control step evaluates the same joint values several times (pose, Jacobian, gravity, surface
        # observe + step): the last forward kinematics is reused.
        self.keys = [name for name, *_ in self.joints]
        self.memo = (None, None)

    def transforms(self, positions):
        key = tuple(positions.get(name, 0) for name in self.keys)
        memo = self.memo
        if memo[0] == key:
            return memo[1]
        result = self.compute(positions)
        self.memo = (key, result)
        return result

    def compute(self, positions):
        transforms = {self.root: np.eye(4)}
        axes = {}
        pending = list(self.joints)
        while pending:
            for joint in list(pending):
                name, parent, child, fixed, axis, kind, mimic = joint
                if parent not in transforms:
                    continue
                frame = transforms[parent] @ fixed
                axes[name] = (frame[:3, 3], frame[:3, :3] @ axis)
                value = positions.get(name, 0)
                if mimic is not None:
                    value = positions.get(mimic.get('joint'), 0) * float(mimic.get('multiplier', 1))
                    value += float(mimic.get('offset', 0))
                motion = np.eye(4)
                if kind in ('revolute', 'continuous'):
                    motion[:3, :3] = rotation(axis * value)
                elif kind == 'prismatic':
                    motion[:3, 3] = axis * value
                transforms[child] = frame @ motion
                pending.remove(joint)
        return transforms, axes

    def pose_jacobian(self, positions):
        transforms, axes = self.transforms(positions)
        pose = transforms[self.tcp_link].copy()  # callers may edit it; the memoised transforms must stay intact
        jacobian = np.column_stack([
            np.r_[np.cross(axes[name][1], pose[:3, 3] - axes[name][0]), axes[name][1]]
            for name in self.names])
        return pose, jacobian

    def gravity_wrench(self, positions):
        transforms, _ = self.transforms(positions)
        sensor = transforms[self.tcp_link][:3, 3]
        wrench = np.zeros(6)
        for name, mass, cog in self.payload:
            center = (transforms[name] @ np.r_[cog, 1])[:3]
            force = np.array([0, 0, -9.81 * mass])
            wrench += np.r_[force, np.cross(center - sensor, force)]
        return wrench

    def gravity_torques(self, positions):
        transforms, axes = self.transforms(positions)
        centres = np.einsum('bij,bj->bi', np.array([transforms[link] for link, _, _ in self.bodies]), self.centres)[:, :3]
        points = np.array([axes[name][0] for name in self.names])
        directions = np.array([axes[name][1] for name in self.names])
        lever = centres[None] - points[:, None]
        # Gravity (0, 0, -9.81 m) on each body downstream of a joint: -axis . (lever x force) = 9.81 m (a_x l_y - a_y l_x)
        moment = directions[:, 0, None] * lever[..., 1] - directions[:, 1, None] * lever[..., 0]
        return 9.81 * (self.below * self.masses * moment).sum(1)
