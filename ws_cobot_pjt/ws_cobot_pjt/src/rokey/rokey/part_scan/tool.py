"""Whether the probe can touch a point along its normal: the rest of the tool clear of the scene (ToolClearance) and
the pose reachable with the arm clear too (Reach). BASE mm throughout."""
import numpy as np
from rcl_interfaces.srv import GetParameters

from .kinematics import RobotChain
from ..rotations import align


def scene_points(vehicle, pose, spacing, rack):
    """Obstacle points: the whole vehicle at `pose` and the probe rack plate (28 x 60 mm, 13-16 mm below the docked
    tip pose `rack`: x, y, z, yaw)."""
    points, _ = vehicle.sample(spacing)
    x, y, z = np.meshgrid(np.arange(-14, 15, spacing), np.arange(-30, 31, spacing), (-16, -13))
    c, s = np.cos(np.radians(rack[3])), np.sin(np.radians(rack[3]))
    plate = np.c_[x.ravel() * c - y.ravel() * s + rack[0], x.ravel() * s + y.ravel() * c + rack[1], z.ravel() + rack[2]]
    return np.concatenate([points @ pose[:3, :3].T + pose[:3, 3], plate])


def robot_chain(robot, tcp_profile, settings):
    """RobotChain of the running robot_state_publisher's URDF and the session's ProbeTCP; spun on the
    RobotServices executor that owns the program node."""
    node = robot.node
    client = node.create_client(GetParameters, '/dsr01/robot_state_publisher/get_parameters')
    try:
        if not client.wait_for_service(timeout_sec=5.):
            raise TimeoutError('robot_state_publisher parameter service unavailable')
        future = client.call_async(GetParameters.Request(names=['robot_description']))
        robot.executor.spin_until_future_complete(future, timeout_sec=5.)
        return RobotChain(future.result().values[0].string_value, tcp_profile, settings)
    finally:
        node.destroy_client(client)


class ToolClearance:
    """The tool body as flat-ended cylinders along its axis from the tip (`cylinders`: [from, to, radius], the ball
    excluded) clear of the obstacle points, the robot base column (`column`: radius, height) and the table
    (`table_z` including its margin); arm links as capsules clear of the obstacle points and the table."""

    def __init__(self, obstacles, cylinders, column, table_z):
        self.points, self.cylinders, self.column, self.table = obstacles, np.asarray(cylinders, float), column, table_z

    def clear(self, tip, axis):
        """`axis`: unit direction from the tip up the tool (the outward normal at a touch)."""
        rim = self.cylinders[:, 2] * np.sqrt(max(0., 1 - axis[2] ** 2))
        if np.any(tip[2] + self.cylinders[:, :2] * axis[2] - rim[:, None] <= self.table):
            return False
        for a, b, r in self.cylinders:
            samples = tip + np.arange(a, b + 10, 10)[:, None] * axis
            if np.any((np.hypot(samples[:, 0], samples[:, 1]) < self.column[0] + r) & (samples[:, 2] < self.column[1] + r)):
                return False
        relative = self.points - tip
        s = relative @ axis
        near = (s >= self.cylinders[0, 0]) & (s <= self.cylinders[-1, 1])
        s, relative = s[near], relative[near]
        radial = np.linalg.norm(relative - s[:, None] * axis, axis=1)
        return not any(np.any((s >= a) & (s <= b) & (radial < r)) for a, b, r in self.cylinders)

    def capsule_clear(self, a, b, radius):
        if min(a[2], b[2]) - radius <= self.table:
            return False
        delta = b - a
        t = np.clip((self.points - a) @ delta / max(delta @ delta, 1e-9), 0, 1)
        return not np.any(np.linalg.norm(self.points - a - t[:, None] * delta, axis=1) < radius)


class Reach:
    """Local URDF IK seeded at HOME (the HOME configuration) of the probe TCP at a tip point with the tool axis on
    the normal, turned the shortest way from the HOME orientation as surface_follow does; the arm links between the
    joints (capsules of `arm_radii`) clear of the obstacles."""

    def __init__(self, chain, home, clearance, arm_radii):
        self.chain, self.home, self.clearance, self.radii = chain, home, clearance, arm_radii
        self.frame = chain.forward(home)[0][:3, :3]
        self.failures = dict(tool_collision=0, ik=0, arm_collision=0)

    def reachable(self, tip, normal):
        q = self.chain.solve(tip / 1000, align(self.frame, normal, np.pi), self.home)
        if q is None:
            return False
        joints = self.chain.forward(q)[2] * 1000
        return all(self.clearance.capsule_clear(a, b, r) for a, b, r in zip(joints[1:-1], joints[2:], self.radii))

    def pose(self, tip, rotation, seed):
        """Continuous full-pose IK and clearance, including the intermediate wrist/tool orientation."""
        if not self.clearance.clear(tip, -rotation[:, 2]):
            self.failures['tool_collision'] += 1
            return None
        q = self.chain.solve(tip / 1000, rotation, seed)
        if q is None:
            self.failures['ik'] += 1
            return None
        joints = self.chain.forward(q)[2] * 1000
        if not all(self.clearance.capsule_clear(a, b, r)
                   for a, b, r in zip(joints[1:-1], joints[2:], self.radii)):
            self.failures['arm_collision'] += 1
            return None
        return q
