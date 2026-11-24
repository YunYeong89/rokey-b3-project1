"""Offline control/stream contracts. No ROS initialization, Docker, Gazebo or robot motion."""
from pathlib import Path
import sys
import time
from types import SimpleNamespace
import unittest

import numpy as np

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))
sys.path.insert(0, str(PACKAGE / 'native'))

from surface_stream import CartesianStream
from normal_force import NormalForceServo
from collision_groups import fixed_probe_assembly


DOWN = np.diag([1., -1., -1.])


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
