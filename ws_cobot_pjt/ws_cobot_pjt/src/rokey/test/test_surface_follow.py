"""Offline control/stream contracts. No ROS initialization, Docker, Gazebo or robot motion."""
from pathlib import Path
import sys
import time
from types import SimpleNamespace
import unittest

import numpy as np

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))
sys.path.insert(0, str(PACKAGE.parents[2] / 'native'))

from rokey.surface_follow.config import Settings
from rokey.surface_follow.controllers import HybridFollower, State
from rokey.surface_follow.geometry import SurfacePath, align
from surface_stream import CartesianStream
from normal_force import NormalForceServo
from collision_groups import fixed_probe_assembly


DOWN = np.diag([1., -1., -1.])


def state(sequence=1, force=1.5):
    return State(sequence, sequence * .025, np.zeros(3), DOWN.copy(),
                 np.array([-.15 * force, 0, force, 0, 0, 0]),
                 np.array([-.15 * force, 0, force, 0, 0, 0]), np.zeros(6))


def path():
    return SurfacePath([[0, 0, .0015], [.03, 0, .0015]], [[0, 0, 1], [0, 0, 1]])


class Chain:
    tcp_link = 'probe_tip'
    names = [f'joint_{i}' for i in range(1, 7)]
    limits = {name: (-10., 10.) for name in names}

    def pose_jacobian(self, joints):
        pose = np.eye(4)
        pose[:3, :3] = DOWN
        return pose, np.eye(6)


def backend():
    chain = Chain()
    q = dict.fromkeys(chain.names, 0.)
    return SimpleNamespace(chain=chain, mode='following', probe=SimpleNamespace(design=SimpleNamespace(attached=True)),
        feedback=q.copy(), nominal=q.copy(), feedback_velocity=q.copy(),
        wrench=np.zeros(6), compensated_wrench=np.zeros(6), ready=lambda: True,
        joint_time=time.monotonic(), sensor_time=time.monotonic())


class SurfaceContracts(unittest.TestCase):
    def test_fixed_path_does_not_reaim_from_a_pose_jump(self):
        theta = np.linspace(0., .4, 30)
        normals = np.c_[np.sin(theta), np.zeros(len(theta)), np.cos(theta)]
        track = SurfacePath(.0415 * normals, normals)
        start = state()
        start.position = track.centres[0] + .0015 * DOWN[:, 2]
        follower = HybridFollower(Settings(), track, start)
        follower.phase, follower.goal = 'TRACK', 1.5
        first = state(2)
        first.position = start.position.copy()
        before = follower.update(first, .025)
        jumped = state(3)
        jumped.position = .0415 * np.array([np.sin(.12), 0., np.cos(.12)]) + .0015 * DOWN[:, 2]
        after = follower.update(jumped, .025)
        self.assertLess(np.linalg.norm(np.subtract(after['normal'], before['normal'])), .01)

    def test_fixed_assembly_retains_scene_and_arm_collisions(self):
        import xml.etree.ElementTree as ET
        robot = ET.fromstring('<model><link name="tool0"><collision name="gripper"/></link>'
                              '<link name="link_3"><collision name="arm"/></link></model>')
        rack = ET.fromstring('<model><link><collision name="rack"/></link></model>')
        probe = ET.fromstring('<model><link><collision name="tip"><surface><friction><ode>'
                              '<mu>0.3</mu></ode></friction></surface></collision></link></model>')
        fixed_probe_assembly(robot, rack, probe)
        mask = lambda collision: int(collision.findtext('surface/contact/collide_bitmask', '65535'))
        tip = probe.find('link/collision')
        self.assertEqual(mask(tip) & mask(robot.find("link[@name='tool0']/collision")), 0)
        self.assertEqual(mask(tip) & mask(rack.find('link/collision')), 0)
        self.assertNotEqual(mask(tip) & mask(robot.find("link[@name='link_3']/collision")), 0)
        self.assertNotEqual(mask(tip) & 65535, 0)  # hemisphere/table use the default group
        self.assertEqual(tip.findtext('surface/friction/ode/mu'), '0.3')

    def test_normal_servo_damps_motion_and_bounds_stored_reference(self):
        servo = NormalForceServo(1.5)
        correction = servo.update(1.5, -.005, 1.5, 0., .01)
        self.assertGreater(correction, 0.)  # inward physical motion is opposed
        for _ in range(100):
            servo.update(0., 0., 1.5, -.0005, .01)
        self.assertTrue(servo.saturated)
        self.assertEqual(servo.velocity, 0.)
        servo = NormalForceServo(1.5)
        for _ in range(10):
            correction = servo.update(4., 0., 1.5, 0., .01)
        self.assertGreater(correction, 0.)  # overload commands outward motion

    def test_tcp_sphere_and_orientation(self):
        sample = state()
        centre = sample.centre(.0015)
        np.testing.assert_allclose(centre - .0015 * np.array([0, 0, 1]), sample.position)
        r = align(DOWN, [1, 0, 0], np.pi)
        np.testing.assert_allclose(r[:, 2], [-1, 0, 0], atol=1e-8)
        np.testing.assert_allclose(r.T @ r, np.eye(3), atol=1e-8)

    def test_force_demand_integral_saturation(self):
        hybrid = HybridFollower(Settings(), path(), state())
        hybrid.goal, hybrid.axial = 1.5, -20.  # the hybrid PI works on the force along the actual tool axis
        for _ in range(80):
            demand = hybrid.force_demand(.025)
        self.assertLessEqual(demand, 2.4)
        self.assertEqual(hybrid.integral, 0.)

    def test_contact_settle_then_overload_unloads(self):
        for cls in (HybridFollower,):
            controller = cls(Settings(), path(), state())
            for i in range(2, 100):
                command = controller.update(state(i), .025)
            self.assertEqual(controller.phase, 'TRACK')
            controller.integral = .8
            for i in range(100, 108):  # a lasting (>= 0.15 s) filtered overload, not a single spike
                overload = state(i, 3.)
                overload.wrench[0] = overload.fast_wrench[0] = 0.  # no feed resistance: not an edge (that backs off)
                command = controller.update(overload, .025)
            self.assertEqual(controller.phase, 'UNLOAD')
            self.assertIsNone(command['force'])
            self.assertEqual(controller.integral, 0.)

    def new_stream(self, method):
        b = backend()
        stream = CartesianStream(b, b.chain)
        pose, j = b.chain.pose_jacobian(b.feedback)
        stream.observe(b.feedback, pose, j, time.monotonic())
        lease = stream.begin(method)
        packet = dict(run_id=lease['run_id'], sequence=1, source_sequence=stream.sequence,
            source_sampled_at=stream.sampled_at, position=[0, 0, 0], rotation=DOWN.tolist(),
            bias=[0.] * 6, radius=.0015, force=None)
        return b, stream, packet

    def test_stream_ownership_staleness_and_watchdog(self):
        with self.assertRaises(ValueError):
            self.new_stream('admittance')  # only the hybrid stream exists
        b, stream, packet = self.new_stream('hybrid')
        with self.assertRaises(ValueError):
            stream.accept({**packet, 'run_id': 'different'})
        with self.assertRaises(ValueError):
            stream.accept({**packet, 'force': 9.})
        with self.assertRaises(ValueError):
            stream.accept({**packet, 'source_sampled_at': time.monotonic() - 1})
        stream.accept(packet)
        with self.assertRaises(ValueError):
            stream.accept(packet)
        self.assertFalse(np.shares_memory(stream.velocity, stream.angular_velocity))
        now = time.monotonic() + .2
        b.joint_time = now
        pose, j = b.chain.pose_jacobian(b.feedback)
        stream.step(b.feedback, pose, j, .01, now)
        self.assertFalse(stream.active)
        self.assertIn('watchdog', stream.reason)
        self.assertEqual(b.mode, 'holding')

    def test_hybrid_inner_force_direction_and_force_interlock(self):
        b, stream, packet = self.new_stream('hybrid')
        stream.accept({**packet, 'force': 1.5})
        initial = stream.reference.copy()
        pose, j = b.chain.pose_jacobian(b.feedback)
        stream.step(b.feedback, pose, j, .01, time.monotonic())
        self.assertLess(stream.reference[2], initial[2])
        b.compensated_wrench[2] = 9.
        stream.observe(b.feedback, pose, j, time.monotonic())
        stream.step(b.feedback, pose, j, .01, time.monotonic())
        self.assertTrue(stream.active)  # one control step over 8 N is a contact spike, not yet an overload
        stream.observe(b.feedback, pose, j, time.monotonic())
        stream.step(b.feedback, pose, j, .01, time.monotonic())
        self.assertFalse(stream.active)
        self.assertIn('8 N', stream.reason)

    def test_wrench_referenced_to_probe_tcp(self):
        class OffsetChain(Chain):
            def pose_jacobian(self, joints):
                pose, j = super().pose_jacobian(joints)
                pose[2, 3] = .1
                return pose, j
        b = backend()
        b.wrench[0] = b.compensated_wrench[0] = 1.
        stream = CartesianStream(b, OffsetChain())
        old_pose, old_j = b.chain.pose_jacobian(b.feedback)
        stream.observe(b.feedback, old_pose, old_j, time.monotonic())
        np.testing.assert_allclose(stream.state['position'], [0, 0, .1])
        np.testing.assert_allclose(stream.state['fast_wrench'][3:], [0, -.1, 0])


if __name__ == '__main__':
    unittest.main()
