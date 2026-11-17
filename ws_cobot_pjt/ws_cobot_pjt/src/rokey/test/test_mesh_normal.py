"""Small offline checks for geometry, TCP pivot and force-independent commands. No robot/ROS startup."""
from pathlib import Path
import sys
import threading
import time
import tempfile
from copy import deepcopy
from types import SimpleNamespace
import unittest

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))
sys.path.insert(0, str(PACKAGE.parents[2] / 'native'))

from rokey.part_scan.mesh_surface import MeshSurface, MeshGrid, TriangleTree
from rokey.part_scan.trajectory import PoseSpan, PoseTrajectory
from rokey.part_scan.mesh_motion import ScanTransfers
from rokey.part_scan.mesh_cache import MeshPlanCache
from rokey.part_scan.mesh_plan import MeshPath
from rokey.part_scan.cloud_plan import CloudSurface, CloudGrid, CloudReach, CloudNormalPlan
from rokey.part_scan.progress_watch import ProgressWatch
from surface_stream import CartesianStream

DOWN = np.diag([1., -1., -1.])
SETTINGS = yaml.safe_load((PACKAGE / 'config/robot.yaml').read_text())['real_scan']['mesh_normal']


class Chain:
    tcp_link = 'probe_tip'
    names = [f'joint_{index}' for index in range(1, 7)]
    limits = dict.fromkeys(names, (-10., 10.))

    def pose_jacobian(self, joints):
        pose = np.eye(4)
        values = np.array([joints[name] for name in self.names])
        pose[:3, 3] = values[:3]
        pose[:3, :3] = Rotation.from_rotvec(values[3:]).as_matrix() @ DOWN
        return pose, np.eye(6)


def stream(force):
    chain = Chain()
    q = dict.fromkeys(chain.names, 0.)
    now = time.monotonic()
    backend = SimpleNamespace(chain=chain, mode='following',
        probe=SimpleNamespace(design=SimpleNamespace(attached=True)),
        feedback=q.copy(), nominal=q.copy(), feedback_velocity=q.copy(),
        wrench=np.full(6, force), compensated_wrench=np.full(6, force),
        ready=lambda: False,  # Legacy force readiness must not gate the position controller.
        nominal_available=True, nominal_time=now, joint_time=now, sensor_time=now - 100.,
        clock=time.monotonic, closed=threading.Event())
    result = CartesianStream(backend, chain)
    pose, jacobian = chain.pose_jacobian(q)
    result.observe(q, pose, jacobian, now)
    result.begin('position')
    return result


def packet(controller, position, orientation):
    return dict(run_id=controller.run_id, sequence=1, source_sequence=controller.sequence,
                source_sampled_at=time.monotonic(), position=position, rotation=orientation,
                feedforward=np.zeros(6), linear_speed=.02, linear_acceleration=.04,
                angular_speed=.15, angular_acceleration=.3, joint_speed=.35, joint_acceleration=.7)


class MeshNormalContracts(unittest.TestCase):
    def test_ball_height_does_not_cut_a_step_smoothed_out_of_the_normals(self):
        x, y = np.meshgrid(np.arange(-4., 4.01, .25), np.arange(-6., 6.01, .25))
        points = np.c_[x.ravel(), y.ravel(), np.where(y.ravel() >= 0., 1.2, 0.)]
        grid = CloudGrid(CloudSurface(points, 'bonnet', 1.), [0., 0., 1.], SETTINGS['pointcloud'])
        uv = np.c_[np.array([-.75, -.5, -.25, 0., .25]), np.zeros(5)]
        positions, normals, valid = grid.sample((uv - grid.origin) / grid.step - .5)
        self.assertTrue(valid.all())
        for position, normal in zip(positions, normals):
            centre = position + normal
            self.assertGreaterEqual(np.linalg.norm(points - centre, axis=1).min(), 1. - 1e-9)

    def test_position_stiffness_increases_restoration_with_a_bounded_offset(self):
        def run(scale, force):
            s = stream(force)
            command = packet(s, [.002, 0., 0.], DOWN)
            command.update(position_stiffness_scale=scale, position_feedback_limit=.0005)
            s.accept(command)
            for _ in range(60):
                s.backend.joint_time = s.backend.nominal_time = time.monotonic()
                s.step(s.backend.feedback, None, None, .01, time.monotonic())
            self.assertLessEqual(np.linalg.norm(s.position_controller.tracking_offset), .0005 + 1e-12)
            return np.array(list(s.q.values()))
        baseline, stronger = run(1., 0.), run(3., 0.)
        self.assertGreater(stronger[0], baseline[0])
        np.testing.assert_array_equal(stronger, run(3., 1000000.))

    def test_stall_watch_requires_persistent_error_and_no_actual_motion(self):
        blocked = ProgressWatch(5., .2, .8)
        moving = ProgressWatch(5., .2, .8)
        settled = ProgressWatch(5., .2, .8)
        for second in range(7):
            stuck = blocked.blocked(second, np.zeros(3), .9)
            self.assertFalse(moving.blocked(second, np.array([second, 0., 0.]), .9))
            self.assertFalse(settled.blocked(second, np.zeros(3), .1))
        self.assertTrue(stuck)

    def test_cloud_grid_estimates_normals_and_preserves_large_openings_without_a_mesh(self):
        x, y = np.meshgrid(np.linspace(-30., 30., 121), np.linspace(-30., 30., 121))
        points = np.c_[x.ravel(), y.ravel(), 100. + .1 * x.ravel() + .2 * y.ravel()]
        surface = CloudSurface(points, 'bonnet')
        self.assertFalse(hasattr(surface, 'triangles'))
        grid = CloudGrid(surface, [0., 0., 1.], SETTINGS['pointcloud'])
        cells = (np.array([[0., 0.]]) - grid.origin) / grid.step - .5
        positions, normals, valid = grid.sample(cells)
        self.assertTrue(valid[0])
        expected = np.array([-.1, -.2, 1.])
        np.testing.assert_allclose(normals[0], expected / np.linalg.norm(expected), atol=.02)
        np.testing.assert_allclose(positions[0, :2], 0., atol=1e-10)
        keep = (np.abs(points[:, 0]) > 8.) | (np.abs(points[:, 1]) > 8.)
        hole = CloudGrid(CloudSurface(points[keep], 'bonnet'), [0., 0., 1.], SETTINGS['pointcloud'])
        cells = (np.array([[0., 0.]]) - hole.origin) / hole.step - .5
        self.assertFalse(hole.sample(cells)[2][0])

    def test_cloud_planner_uses_sparse_ik_without_collision_or_transit_checks(self):
        class CloudChain:
            calls = 0

            def forward(self, q):
                pose = np.eye(4)
                pose[:3, :3] = DOWN
                return pose, np.eye(6), np.empty((0, 3))

            def solve(self, position, rotation, seed):
                self.calls += 1
                return np.r_[position, np.zeros(3)]

        x, y = np.meshgrid(np.arange(-30., 31.), np.arange(-30., 31.))
        points = np.c_[x.ravel(), y.ravel(), np.full(x.size, 100.)]
        scan = deepcopy(yaml.safe_load((PACKAGE / 'config/robot.yaml').read_text())['real_scan'])
        settings = scan['mesh_normal']
        for key in ('checkpoint_max_mm', 'normal_step_deg', 'normal_interp_error_deg', 'position_interp_error_mm'):
            settings[key] = settings['pointcloud'][key]
        settings['validation_step_mm'], settings['validation_angle_deg'] = 10., 15.
        chain = CloudChain()
        reach = CloudReach(chain, np.zeros(6))
        plan = CloudNormalPlan(CloudSurface(points, 'bonnet'), np.array([[0., 0., 1.]]), reach, scan, 200.)
        self.assertEqual(len(plan.paths), 2)
        self.assertLess(chain.calls, sum(path.trajectory.dense_count for path in plan.paths))
        self.assertFalse(plan.planning_checks['tool_collision'])
        self.assertFalse(plan.planning_checks['transit'])

    def test_cached_stroke_preserves_poses_retimes_speed_and_reuses_entry_validation(self):
        class Reach:
            home = np.zeros(6)
            calls = 0
            chain = SimpleNamespace(description='test robot', tcp=np.eye(4),
                                    forward=lambda q: (np.eye(4), np.eye(6), np.empty((0, 3))))
            clearance = SimpleNamespace(table=0., rack=np.zeros(3), rack_rotation=np.eye(3), ball_radius=1.)

            def pose(self, position, rotation, seed):
                self.calls += 1
                return np.r_[position / 1000, np.zeros(3)]

        reach = Reach()
        points = np.c_[np.linspace(0., 40., 161), np.zeros((161, 2))]
        rotations = np.repeat(DOWN[None], len(points), axis=0)
        trajectory = PoseTrajectory(points, rotations, SETTINGS)
        trajectory.validate(reach, np.zeros(6), 5., 5.)
        restored = PoseTrajectory.from_cache(trajectory.cache_data(), SETTINGS, 3.)
        for progress in (0., .3, .7, 1.):
            a = trajectory.sample(progress * trajectory.duration)
            b = restored.sample(progress * restored.duration)
            np.testing.assert_allclose(a[0], b[0], atol=1e-10)
            np.testing.assert_allclose(a[1], b[1], atol=1e-10)
        self.assertGreater(restored.duration, trajectory.duration)
        before = reach.calls
        restored.entry(reach, np.zeros(6), 3., 5.)
        self.assertEqual(reach.calls - before, 1)

        scan = deepcopy(yaml.safe_load((PACKAGE / 'config/robot.yaml').read_text())['real_scan'])
        scan['speed_mm_s'] = 5.
        surface = SimpleNamespace(triangles=np.array([[[0., 0., 0.], [40., 0., 0.], [0., 10., 0.]]]),
                                  target=np.array([True]), part='bonnet')
        normals = np.repeat([[0., 0., 1.]], len(points), axis=0)
        plan = SimpleNamespace(paths=[MeshPath(points, normals, np.array([0., 0., 1.]), trajectory)],
                               diagnostics=[], rejected=[])
        with tempfile.TemporaryDirectory() as directory:
            cache = MeshPlanCache(Path(directory), surface, [[0., 0., 1.]], reach, scan, 200.)
            cache.save(plan)
            scan['speed_mm_s'] = 3.
            faster_load = MeshPlanCache(Path(directory), surface, [[0., 0., 1.]], reach, scan, 200.)
            self.assertEqual(cache.key, faster_load.key)
            self.assertEqual(len(faster_load.load().paths), 1)
            surface.triangles = surface.triangles + [0., 0., 1.]
            changed = MeshPlanCache(Path(directory), surface, [[0., 0., 1.]], reach, scan, 200.)
            self.assertNotEqual(cache.key, changed.key)
            self.assertIsNone(changed.load())

    def test_ordinary_transfer_speeds_are_independent_of_detection_speed(self):
        class Probe:
            speeds = []

            def read(self):
                return np.array([400., 0., 150., 0., 180., 0.]), np.zeros(6)

            def move(self, position, rotation, speed, acc):
                self.speeds.append(speed)

        config = yaml.safe_load((PACKAGE / 'config/robot.yaml').read_text())
        scan = deepcopy(config['real_scan'])
        scan['speed_mm_s'] = 3.
        probe = Probe()
        program = SimpleNamespace(probe=probe, config=config, safe_z=lambda: 220.)
        transfers = ScanTransfers(program, scan, DOWN)
        trajectory = SimpleNamespace(sample=lambda t: (np.array([440., 0., 145.]), DOWN, np.zeros(6)))
        transfers.approach(trajectory)
        transfers.retract()
        self.assertEqual(probe.speeds, [30., 100., 30., 30., 30., 10., 10., 30., 30., 30.])
        self.assertNotIn(scan['speed_mm_s'], probe.speeds)

    def test_tool_cylinder_has_flat_ends_and_still_detects_real_intersections(self):
        plane = np.array([[[-100., -100., 0.], [100., -100., 0.], [0., 100., 0.]]])
        tree = TriangleTree(plane)
        tip, normal = np.zeros(3), np.array([0., 0., 1.])
        self.assertFalse(tree.clear(np.array([0., 0., 14.]), 23.))
        self.assertTrue(tree.cylinder_clear(tip, normal, 14., 67., 23.))
        self.assertFalse(TriangleTree(plane + [0., 0., 20.]).cylinder_clear(tip, normal, 14., 67., 23.))
        self.assertTrue(TriangleTree(plane + [100., 0., 20.]).cylinder_clear(tip, normal, 14., 67., 3.))

    def test_wrench_magnitude_and_sensor_age_cannot_change_joint_commands(self):
        outputs = []
        for force in (0., 1000000.):
            s = stream(force)
            s.accept(packet(s, [.002, 0., 0.], Rotation.from_rotvec([0., .05, 0.]).as_matrix() @ DOWN))
            commands = []
            for _ in range(12):
                commands.append(list(s.step(s.backend.feedback, None, None, .01, time.monotonic()).values()))
            self.assertTrue(s.active)
            self.assertFalse(s.control['force_feedback_used'])
            outputs.append(commands)
        np.testing.assert_array_equal(outputs[0], outputs[1])

    def test_rotation_uses_fixed_tcp_instead_of_ball_centre(self):
        s = stream(0.)
        s.accept(packet(s, [0., 0., 0.], Rotation.from_rotvec([0., .1, 0.]).as_matrix() @ DOWN))
        for _ in range(12):
            s.step(s.backend.feedback, None, None, .01, time.monotonic())
        np.testing.assert_array_equal(s.position_controller.position, np.zeros(3))
        self.assertGreater(np.linalg.norm(s.position_controller.orientation - DOWN), .001)
        np.testing.assert_array_equal([s.q[name] for name in s.chain.names[:3]], np.zeros(3))

    def test_curvature_keeps_more_checkpoints_and_quintic_stops_smoothly(self):
        x = np.linspace(0., 60., 241)
        points = np.c_[x, np.zeros_like(x), np.zeros_like(x)]
        flat_rotations = np.repeat(DOWN[None], len(x), axis=0)
        # Equal endpoint normals with an interior turn must still retain interior checkpoints.
        curved_rotations = Rotation.from_euler('y', (.3 * np.sin(np.pi * x / 60))[:, None]).as_matrix() @ DOWN
        flat, curved = PoseSpan(points, flat_rotations, SETTINGS), PoseSpan(points, curved_rotations, SETTINGS)
        self.assertGreater(len(curved.knots), len(flat.knots))
        self.assertLessEqual(curved.max_angle_error_deg, SETTINGS['normal_interp_error_deg'])
        s, _, _ = curved.validation_samples()
        curved.retime(s, np.zeros((len(s), 6)), 5.)
        np.testing.assert_allclose(curved.sample(0.)[2], 0., atol=1e-12)
        np.testing.assert_allclose(curved.sample(curved.duration)[2], 0., atol=1e-12)

    def test_mesh_queries_respect_occluding_parts_and_return_surface_normals(self):
        triangles = np.array([[[0., 0., 0.], [10., 0., 0.], [0., 10., 0.]],
                              [[0., 0., 2.], [4., 0., 2.], [0., 4., 2.]]])
        vehicle = SimpleNamespace(triangles=triangles, parts=np.array(['bonnet', 'body']))
        surface = MeshSurface(vehicle, 'bonnet', np.eye(4), 35.)
        grid = MeshGrid(surface, np.array([0., 0., 1.]), 1., 0.)
        points = np.array([[1., 1., 0.], [6., 1., 0.], [8., 8., 0.]])
        uv = np.c_[points @ grid.u, points @ grid.v]
        positions, normals, faces = grid.sample((uv - grid.origin) / grid.step - .5)
        self.assertEqual(faces.tolist(), [-1, 0, -1])
        np.testing.assert_allclose(positions[1], points[1])
        np.testing.assert_allclose(normals[1], [0., 0., 1.])


if __name__ == '__main__':
    unittest.main()
