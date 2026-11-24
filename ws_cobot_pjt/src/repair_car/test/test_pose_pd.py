"""Focused offline PD contracts. No ROS initialization or robot motion."""
import csv
import json
from pathlib import Path
import tempfile
import time
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from test_mesh_normal import DOWN, packet, stream
from pose_pd import PDSettings, VectorPD


class PosePDContracts(unittest.TestCase):
    def test_derivative_brakes_motion_and_saturated_output_has_no_integral_memory(self):
        pd = VectorPD(.03)
        output = pd.update(np.zeros(3), np.zeros(3), np.array([.1, 0., 0.]), .01,
                           1., .04, .0005, .002)
        self.assertLess(pd.d[0], 0.)
        self.assertLess(output[0], 0.)
        for _ in range(60):
            pd.update(np.ones(3), np.zeros(3), np.zeros(3), .01, 1., .04, .0005, .002)
        self.assertTrue(pd.limited)
        self.assertLessEqual(np.linalg.norm(pd.offset), .0005 + 1e-12)
        for _ in range(80):
            pd.update(np.zeros(3), np.zeros(3), np.zeros(3), .01, 1., .04, .0005, .002)
        np.testing.assert_allclose(pd.offset, 0., atol=1e-12)

    def test_actual_pose_and_velocity_change_pd_even_with_unchanged_nominal_command(self):
        s = stream(0., pd={'enabled': True})
        command = packet(s, np.zeros(3), DOWN)
        command['position_stiffness_scale'] = 3.
        s.accept(command)
        s.backend.feedback['joint_1'] = .001
        s.backend.feedback['joint_4'] = .02
        s.backend.feedback_velocity['joint_4'] = .1
        s.step(s.backend.feedback, None, None, .01, time.monotonic())
        self.assertLess(s.control['pd_error'][0], 0.)
        self.assertLess(s.control['pd_error'][3], 0.)
        self.assertLess(s.control['pd_d'][3], 0.)
        self.assertLess(s.q['joint_4'], 0.)
        self.assertTrue(s.control['pd_enabled'])

    def test_pd_joint_commands_are_independent_of_measured_force(self):
        outputs = []
        for force in (0., 1000000.):
            s = stream(force, pd={'enabled': True})
            command = packet(s, [.002, 0., 0.], Rotation.from_rotvec([0., .05, 0.]).as_matrix() @ DOWN)
            command['position_stiffness_scale'] = 3.
            s.accept(command)
            samples = []
            for _ in range(20):
                s.backend.joint_time = s.backend.nominal_time = time.monotonic()
                samples.append(list(s.step(s.backend.feedback, None, None, .01, time.monotonic()).values()))
                json.dumps(s.control)
            outputs.append(samples)
        np.testing.assert_array_equal(outputs[0], outputs[1])

    def test_combined_reference_and_pd_obey_angular_limits_and_keep_tcp_pivot(self):
        s = stream(0., pd={'enabled': True})
        command = packet(s, np.zeros(3), Rotation.from_rotvec([0., .2, 0.]).as_matrix() @ DOWN)
        command['angular_speed'], command['angular_acceleration'] = np.radians([16., 16.])
        s.accept(command)
        previous = np.zeros(3)
        for _ in range(50):
            s.backend.joint_time = s.backend.nominal_time = time.monotonic()
            s.step(s.backend.feedback, None, None, .01, time.monotonic())
            controller = s.position_controller
            self.assertLessEqual(np.linalg.norm(controller.command_omega), np.radians(16.) + 1e-12)
            self.assertLessEqual(np.linalg.norm(controller.command_omega - previous), np.radians(16.) * .01 + 1e-12)
            previous = controller.command_omega.copy()
            np.testing.assert_array_equal(controller.command_position, np.zeros(3))
            self.assertLessEqual(np.linalg.norm(controller.pd.orientation.offset), np.radians(2.) + 1e-12)

    def test_invalid_pd_configuration_does_not_activate_a_session(self):
        s = stream(0.)
        self.assertFalse(s.control['pd_enabled'])
        self.assertTrue(stream(0., pd={'enabled': True}).control['pd_enabled'])
        s.halt('test')
        with self.assertRaises(ValueError):
            s.begin('position', {'position_kd_s': -1.})
        self.assertFalse(s.active)
        with self.assertRaises(ValueError):
            PDSettings.parse({'ki': 1.})

    def test_pd_csv_and_endpoint_summary_keep_units_and_columns_aligned(self):
        from repair_car.report.mesh_report import MeshReport
        row = [0., 1, 'stroke', *([0.] * 31)]
        # enabled, saturated, 4 pose-offset vectors, 1 velocity-error vector (mm/deg units in CSV).
        row += [1, 0, .2, 0., 0., .1, 0., 0., *([0.] * 24)]
        row += [0.] * 9
        with tempfile.TemporaryDirectory() as directory:
            report = MeshReport(Path(directory), 'pd')
            report.metadata = {'paths': [{'duration_s': 10.}]}
            report.save([row], True, [{'path': 1, 'completed': True, 'tracking_pause_s': 2.}])
            with (report.directory / 'samples.csv').open() as output:
                records = list(csv.reader(output))
            self.assertEqual(len(records[0]), len(records[1]))
            result = json.loads((report.directory / 'result.json').read_text())
            self.assertFalse(result['pd']['integral_enabled'])
            self.assertEqual(result['pd']['paths']['1']['endpoint_mean_position_error_mm'], [.2, 0., 0.])
            self.assertEqual(result['timing']['planned_strokes_s'], 10.)
            self.assertEqual(result['timing']['executed_strokes'][0]['tracking_pause_s'], 2.)

    def test_stroke_timing_separates_tracking_pauses_and_endpoint_wait(self):
        from repair_car.inspection.part_scan.scan_timing import StrokeTiming
        clock = [0.]
        timing = StrokeTiming(1, 10., clock=lambda: clock[0])
        timing.observe(1., True, True, False, 1.)
        timing.observe(3., False, True, False, 1.)
        timing.observe(4., False, False, False, 1.)
        timing.observe(13., True, True, False, 10.)
        timing.observe(13.5, True, True, True, 10.)
        clock[0] = 13.5
        timing.finish()
        result = timing.export()
        self.assertEqual(result['wall_s'], 13.5)
        self.assertEqual(result['advanced_s'], 10.)
        self.assertEqual(result['tracking_pause_s'], 3.)
        self.assertEqual(result['position_pause_s'], 3.)
        self.assertEqual(result['orientation_pause_s'], 1.)
        self.assertEqual(result['endpoint_wait_s'], .5)


if __name__ == '__main__':
    unittest.main()
