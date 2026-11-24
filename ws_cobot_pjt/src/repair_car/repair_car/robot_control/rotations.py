"""Rotation helpers (BASE frame): Doosan ZYZ Euler angles in degrees and axis-angle rotations."""
import numpy as np


def rotation_zyz(angles):
    a, b, c = np.deg2rad(angles)
    rz = lambda v: np.array([[np.cos(v), -np.sin(v), 0], [np.sin(v), np.cos(v), 0], [0, 0, 1]])
    ry = np.array([[np.cos(b), 0, np.sin(b)], [0, 1, 0], [-np.sin(b), 0, np.cos(b)]])
    return rz(a) @ ry @ rz(c)


def angles_zyz(matrix, previous):
    """Keep equivalent Euler representations close, including beta=0/pi."""
    b = np.arccos(np.clip(matrix[2, 2], -1, 1))
    if abs(np.sin(b)) > 1e-7:
        a = np.arctan2(matrix[1, 2], matrix[0, 2])
        c = np.arctan2(matrix[2, 1], -matrix[2, 0])
    else:
        a = np.deg2rad(previous[0])
        if matrix[2, 2] > 0:
            c = np.arctan2(matrix[1, 0], matrix[0, 0]) - a
        else:
            c = a - np.arctan2(-matrix[1, 0], -matrix[0, 0])
    primary = np.rad2deg([a, b, c])
    candidates = [primary, primary * [1, -1, 1] + [180, 0, 180]]
    candidates = [value + 360 * np.round((np.array(previous) - value) / 360) for value in candidates]
    return min(candidates, key=lambda value: np.linalg.norm(value - previous)).tolist()


def axis_angle(axis, angle):
    x, y, z = axis
    k = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + np.sin(angle) * k + (1 - np.cos(angle)) * k @ k


def unit(value):
    value = np.asarray(value, dtype=float)
    length = np.linalg.norm(value)
    if length < 1e-9:
        raise ValueError('Zero direction vector')
    return value / length


def align(rotation, normal, max_angle):
    """Shortest incremental alignment of TOOL +Z with inward normal; retains tool yaw."""
    source, target = rotation[:, 2], -unit(normal)
    cross = np.cross(source, target)
    angle = np.arctan2(np.linalg.norm(cross), np.clip(source @ target, -1, 1))
    if angle < 1e-9:
        return rotation.copy()
    axis = unit(cross) if np.linalg.norm(cross) > 1e-8 else rotation[:, 0]
    return axis_angle(axis, min(angle, max_angle)) @ rotation
