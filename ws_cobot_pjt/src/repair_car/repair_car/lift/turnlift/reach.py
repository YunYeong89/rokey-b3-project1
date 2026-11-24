"""Planning-level reach and clearance of the probe on the M0609 (device.yaml robot / probe), independent of the robot
package: spherical wrist, elbow-up arm in the vertical plane through the wrist centre; the controller's IK and the
real link meshes decide in the end. BASE mm."""
import numpy as np

from .mesh import box_triangles, sample


class SimpleArm:
    def __init__(self, robot, probe):
        self.shoulder = np.array([0., 0., robot['shoulder_z_mm']])
        self.a2, self.a3 = robot['upper_mm'], robot['fore_mm']
        self.back = probe['tcp_mm'] + robot['wrist_mm']    # tip -> wrist centre along the tool
        elbow = np.radians(robot['elbow_min_deg'])
        self.near, self.far = np.sqrt(self.a2 ** 2 + self.a3 ** 2 - 2 * self.a2 * self.a3 * np.cos(elbow)), self.a2 + self.a3
        self.radii = robot['arm_radii_mm']

    def links(self, tip, axis):
        """[(start, end, radius)] of the upper arm and forearm for a tip touching along `axis` (unit, from the tip up
        the tool); None when the wrist centre is out of reach."""
        wrist = tip + self.back * axis
        d = wrist - self.shoulder
        rho, dist = np.hypot(d[0], d[1]), np.linalg.norm(d)
        if not self.near <= dist <= self.far:
            return None
        heading = d[:2] / rho if rho > 1e-6 else np.array([1., 0.])
        phi = np.arctan2(d[2], rho) + np.arccos((self.a2 ** 2 + dist ** 2 - self.a3 ** 2) / (2 * self.a2 * dist))
        elbow = self.shoulder + self.a2 * np.r_[np.cos(phi) * heading, np.sin(phi)]
        return [(self.shoulder, elbow, self.radii[0]), (elbow, wrist, self.radii[1])]


class Clearance:
    """Tool cylinders (flat ends, from the tip, ball excluded) and arm capsules clear of the obstacle points, the robot
    base column (tool only) and the table."""

    def __init__(self, points, probe, robot, table_z, spacing):
        rack = np.array(robot['rack_box_mm']).reshape(2, 3)
        self.points = np.concatenate([points, sample(box_triangles(*rack), spacing)[0]])
        self.tool, self.column, self.table = np.asarray(probe['tool'], float), robot['base_column_mm'], table_z

    def tool_clear(self, tip, axis):
        rim = self.tool[:, 2] * np.sqrt(max(0., 1 - axis[2] ** 2))
        if np.any(tip[2] + self.tool[:, :2] * axis[2] - rim[:, None] <= self.table):
            return False
        for a, b, r in self.tool:
            samples = tip + np.arange(a, b + 10, 10)[:, None] * axis
            if np.any((np.hypot(samples[:, 0], samples[:, 1]) < self.column[0] + r) & (samples[:, 2] < self.column[1] + r)):
                return False
        relative = self.points - tip
        s = relative @ axis
        near = (s >= self.tool[0, 0]) & (s <= self.tool[-1, 1])
        radial = np.linalg.norm(relative[near] - s[near, None] * axis, axis=1)
        return not any(np.any((s[near] >= a) & (s[near] <= b) & (radial < r)) for a, b, r in self.tool)

    def capsule_clear(self, a, b, radius):
        if min(a[2], b[2]) - radius <= self.table:
            return False
        delta = b - a
        t = np.clip((self.points - a) @ delta / max(delta @ delta, 1e-9), 0, 1)
        return not np.any(np.linalg.norm(self.points - a - t[:, None] * delta, axis=1) < radius)

    def clear(self, tip, axis, links):
        return self.tool_clear(tip, axis) and all(self.capsule_clear(a, b, r) for a, b, r in links)
