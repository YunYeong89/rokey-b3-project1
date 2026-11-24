"""Ball-pivot geometry and local timing contracts, without ROS or robot startup."""
from copy import deepcopy
import time
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from test_mesh_normal import DOWN, SETTINGS, packet, stream
from repair_car.inspection.part_scan.trajectory import PoseSpan
from repair_car.inspection.part_scan.cloud_plan import CloudGrid, CloudSurface
from repair_car.inspection.part_scan.cloud_path import CloudPath
from repair_car.inspection.part_scan.cloud_clearance import CloudProbeClearance


class BallMotionContracts(unittest.TestCase):
    def test_neighbouring_body_cloud_excludes_hidden_bonnet_cells(self):
        x, y = np.meshgrid(np.arange(-12., 13.), np.arange(-12., 13.))
        points = np.c_[x.ravel(), y.ravel(), np.full(x.size, 100.)]
        body = points[(points[:, 0] >= 0.) & (points[:, 1] >= 0.)].copy()
        body[:, 2] += 2.
        grid = CloudGrid(CloudSurface(points, 'bonnet', obstacles=body), [0., 0., 1.], SETTINGS['pointcloud'])
        uv = np.array([[5., -5.], [-6., 6.]])
        cells = np.floor((uv - grid.origin) / grid.step).astype(int)
        self.assertFalse(grid.inner[tuple(cells[0])])
        self.assertTrue(grid.inner[tuple(cells[1])])
        check = CloudProbeClearance(np.array([[0., 0., 0.], [10., 0., 30.]]), [(5., 14., 3.), (14., 67., 23.)])
        self.assertTrue(check.blocked(np.zeros((1, 3)), np.array([[0., 0., 1.]]))[0])
        self.assertFalse(check.blocked(np.zeros((1, 3)), np.array([[0., 0., -1.]]))[0])

    def span(self, centres, rotations):
        settings = deepcopy(SETTINGS)
        settings['ball_radius_mm'] = 1.
        settings.update(position_interpolator='pchip', position_interp_error_mm=.1)
        span = PoseSpan(centres + rotations[:, :, 2], rotations, settings)
        s, _, _ = span.validation_samples()
        span.retime(s, np.zeros((len(s), 6)), 6.)
        return span

    def test_long_flat_stroke_cruises_at_six_and_starts_with_local_acceleration(self):
        x = np.linspace(0., 120., 241)
        centres = np.c_[x, np.zeros_like(x), np.ones_like(x)]
        rotations = np.repeat(DOWN[None], len(x), axis=0)
        span = self.span(centres, rotations)
        self.assertLess(span.duration, 22.)
        self.assertGreater(span.sample(1.)[0][0], 3.)
        self.assertAlmostEqual(np.linalg.norm(span.sample(span.duration / 2)[2][:3]), 6., delta=.01)
        np.testing.assert_allclose(span.sample(span.duration)[2], 0., atol=1e-10)

    def test_rotating_gripper_keeps_ball_on_path_and_feedforward_matches_pivot(self):
        x = np.linspace(0., 60., 241)
        centres = np.c_[x, np.zeros_like(x), np.ones_like(x)]
        rotations = Rotation.from_rotvec(np.c_[np.zeros_like(x), .1 * np.sin(x / 10), np.zeros_like(x)]).as_matrix() @ DOWN
        raw_normals = -rotations[:, :, 2].copy()
        raw_normals[len(x) // 2] = [0., np.sqrt(.5), np.sqrt(.5)]
        filtered = CloudPath.normals(raw_normals, 4., .25, 5.)
        angles = np.degrees(np.arccos(np.clip(np.sum(filtered * raw_normals, axis=1), -1., 1.)))
        self.assertLessEqual(angles.max(), 5. + 1e-8)
        span = self.span(centres, rotations)
        for t in np.linspace(.5, span.duration - .5, 15):
            p, r, feed = span.sample(t)
            centre = p - r[:, 2]
            self.assertAlmostEqual(centre[1], 0.)
            self.assertAlmostEqual(centre[2], 1.)
            velocity = feed[:3] - np.cross(feed[3:], r[:, 2])
            self.assertLessEqual(np.linalg.norm(velocity), 6. + 1e-8)
            self.assertLessEqual(np.linalg.norm(feed[3:]), np.radians(16.) + 1e-8)
            before, after = span.sample(t - 1e-5)[2], span.sample(t + 1e-5)[2]
            self.assertLessEqual(np.linalg.norm((after[3:] - before[3:]) / 2e-5), np.radians(16.) + 1e-5)

    def test_native_rotation_compensates_tcp_to_hold_ball_centre(self):
        s = stream(0., pd={'enabled': True})
        r = Rotation.from_rotvec([0., .1, 0.]).as_matrix() @ DOWN
        # Initial TCP=0, ball centre=-r_ball*z=(0,0,.001).
        centre = np.array([0., 0., .001])
        command = packet(s, centre + .001 * r[:, 2], r)
        command['ball_radius'] = .001
        s.accept(command)
        for _ in range(30):
            s.backend.joint_time = s.backend.nominal_time = time.monotonic()
            s.step(s.backend.feedback, None, None, .01, time.monotonic())
        np.testing.assert_allclose(s.position_controller.position, centre, atol=1e-12)
        self.assertGreater(np.linalg.norm(s.q['joint_1']), 1e-7)


if __name__ == '__main__':
    unittest.main()
