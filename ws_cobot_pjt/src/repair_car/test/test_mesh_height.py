"""Focused offline checks of fixed-axis height planning; no robot motion."""
from types import SimpleNamespace
import unittest

import numpy as np

from repair_car.config import load
from repair_car.inspection.part_scan.height_plan import MeshHeightPlan
from repair_car.inspection.part_scan.mesh_surface import MeshSurface
from repair_car.inspection.part_scan.mesh_surface import MeshGrid
from repair_car.experiments.performance_sample import PerformanceSample


DOWN = np.diag([1., -1., -1.])


class CartesianReach:
    frame = DOWN
    home = np.zeros(6)
    failures = {}

    def __init__(self):
        self.chain = self

    def pose(self, position, rotation, seed):
        return np.r_[position / 1000, np.zeros(3)]

    def forward(self, joints):
        pose = np.eye(4)
        pose[:3, :3], pose[:3, 3] = DOWN, joints[:3]
        return pose, np.eye(6), np.empty((0, 3))


class MeshHeightContracts(unittest.TestCase):
    def test_coupon_plans_only_known_regions_on_a_healthy_plane(self):
        coupon = PerformanceSample()
        settings = load('robot')['performance_scan']
        regions = coupon.planning_regions(settings)
        nominal = coupon.planning_surface(regions=regions)
        surface = MeshSurface(nominal, 'box', np.eye(4), 35.)
        grid = MeshGrid(surface, [0., 0., 1.], .5, 4.)
        xy = np.array([[-60., 10.], [25., 25.], [0., -50.], [-10., 40.]])
        uv = np.c_[xy @ grid.u[:2], xy @ grid.v[:2]]
        points, _, faces = grid.sample((uv - grid.origin) / grid.step - .5)
        self.assertTrue(np.all(faces[:2] >= 0))
        self.assertTrue(np.all(faces[2:] == -1))
        np.testing.assert_allclose(points[:2, 2], coupon.size[2])
        self.assertLess(grid.inner.sum(), 40000)
        custom = [-20., -15., 20., 15.]
        self.assertEqual(coupon.planning_regions(settings, roi=custom), [custom])

    def test_coupon_cloud_and_planning_mesh_are_healthy_inside_the_roi(self):
        coupon = PerformanceSample()
        original = coupon.triangles.copy()
        pose = np.eye(4)
        pose[2, 3] = 17.
        reference = coupon.reference(pose, 10.)
        np.testing.assert_allclose(reference.points[:, 2], coupon.size[2] + 17.)
        nominal = coupon.planning_surface([-20., -15., 20., 15.])
        surface = MeshSurface(nominal, 'box', pose, 35.)
        grid = MeshGrid(surface, [0., 0., 1.], 1., 0.)
        xy = np.array([[0., 0.], [5., -2.], [30., 0.]])
        uv = np.c_[xy @ grid.u[:2], xy @ grid.v[:2]]
        positions, normals, faces = grid.sample((uv - grid.origin) / grid.step - .5)
        self.assertTrue(np.all(faces[:2] >= 0))
        self.assertEqual(faces[2], -1)
        np.testing.assert_allclose(positions[:2, 2], coupon.size[2] + 17.)
        np.testing.assert_allclose(normals[:2], [[0., 0., 1.], [0., 0., 1.]])
        np.testing.assert_array_equal(coupon.triangles, original)

    def test_height_configuration_copies_settings_and_disables_pd(self):
        original = load('robot')['real_scan']
        scan = MeshHeightPlan.configure(original)
        self.assertFalse(scan['mesh_height']['pd']['enabled'])
        self.assertEqual(scan['mesh_height']['position_stiffness_scale'], 1.)
        self.assertEqual(scan['mesh_height']['planning_source'], 'mesh')
        self.assertTrue(original['mesh_height']['pd']['enabled'])
        self.assertEqual(original['mesh_height']['position_stiffness_scale'], 1.)

    def test_curved_mesh_changes_height_while_tool_and_force_axis_stay_vertical(self):
        scan = MeshHeightPlan.configure(load('robot')['real_scan'])
        scan.update(step_mm=2., pitch_mm=6., ik_mm=2., min_path_mm=5.)
        scan['mesh_height'].update(ball_radius_mm=1., tool_tilt_deg=0.)
        x = np.arange(-20., 21.)
        lower = np.c_[x, np.full(len(x), -10.), 100. + .005 * x**2]
        upper = lower.copy()
        upper[:, 1] = 10.
        triangles = np.array([triangle for i in range(len(x) - 1) for triangle in
                              ([lower[i], lower[i + 1], upper[i]],
                               [upper[i], lower[i + 1], upper[i + 1]])])
        vehicle = SimpleNamespace(triangles=triangles, parts=np.full(len(triangles), 'bonnet'))
        surface = MeshSurface(vehicle, 'bonnet', np.eye(4), 35.)
        plan = MeshHeightPlan(surface, np.array([[0., 0., 1.]]), CartesianReach(), scan, 130.)
        self.assertTrue(plan.paths)
        z = []
        for path in plan.paths:
            np.testing.assert_allclose(path.normals, np.tile([0., 0., 1.], (len(path.normals), 1)))
            for elapsed in np.linspace(0., path.trajectory.duration, 30):
                position, rotation, feed = path.trajectory.sample(elapsed)
                np.testing.assert_allclose(rotation[:, 2], [0., 0., -1.], atol=1e-12)
                np.testing.assert_allclose(feed[3:], 0., atol=1e-12)
                self.assertLess(abs(position[2] - 100. - .005 * position[0]**2 - plan.depth), .1)
                z.append(position[2])
        self.assertGreater(np.ptp(z), .5)


if __name__ == '__main__':
    unittest.main()
