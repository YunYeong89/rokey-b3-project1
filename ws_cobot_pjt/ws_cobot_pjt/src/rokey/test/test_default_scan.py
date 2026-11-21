"""Offline checks of low-load crack evidence, uninterrupted strokes, and ROI-only re-planning."""
from copy import deepcopy
from types import SimpleNamespace
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

import numpy as np

from rokey.configs import load
from rokey.part_scan.cloud import HeightMap
from rokey.part_scan.planner import PartPlan
from rokey.part_scan.sampling import PointSampler
from rokey.part_scan.staged_roi import RoiPlan, SurveyRoi
from rokey.part_scan.staged_scan import DefaultScan, StageScraper
from rokey.part_scan.figure_book import FigureBook


AXIS = np.array([0., 0., 1.])


class DefaultScanContracts(unittest.TestCase):
    def setUp(self):
        self.scan = load('robot')['real_scan']
        self.settings = self.scan['default_scan']
        self.grid = SimpleNamespace(ball_at=lambda points: np.full(len(points), 100.),
                                    u=np.array([1., 0., 0.]), v=np.array([0., 1., 0.]))

    def trace(self):
        x = np.arange(0., 30., .1)
        force = .8 + .01 * np.sin(x * 2)
        records = np.c_[x / 5., x, np.zeros_like(x), np.full(len(x), 100.), force,
                        np.tile(AXIS, (len(x), 1)), np.ones(len(x))]
        return SimpleNamespace(records=records, scan={'force_n': .8}, threshold=.5)

    def test_shallow_force_loss_and_depth_only_are_both_candidates(self):
        scraper = self.trace()
        self.assertFalse(SurveyRoi(scraper, [(AXIS, self.grid)], self.settings['roi']).regions)
        scraper.records[100:120, 4] = .05  # crack bridged with essentially no immersion
        scraper.records[210:230, 3] = 99.75  # force-regulated immersion without a force anomaly
        rois = SurveyRoi(scraper, [(AXIS, self.grid)], self.settings['roi'])
        self.assertTrue(any('contact_loss' in r['evidence'] and r['bounds_mm'][0] <= 11 <= r['bounds_mm'][3]
                            for r in rois.regions))
        self.assertTrue(any('geometry' in r['evidence'] and r['bounds_mm'][0] <= 22 <= r['bounds_mm'][3]
                            for r in rois.regions))
        self.assertTrue(any('force' in r['evidence'] for r in rois.regions))

    def test_measurement_never_interrupts_deep_or_high_force_strokes(self):
        scraper = StageScraper.__new__(StageScraper)
        scraper.program = SimpleNamespace(topics=Mock())
        scraper.scan = DefaultScan.survey_settings(self.scan)
        scraper.grid, scraper.path, scraper.threshold = self.grid, 1, .5
        scraper.records, scraper.points, scraper.point_samples, scraper.control = [], [], [], []
        scraper.transport, scraper.next_write = None, float('inf')
        scraper.sampler = PointSampler(self.scan['sample_mm'])
        measure = scraper.measure(AXIS)
        for force in (.01, 100.):
            self.assertFalse(measure(np.array([5., 0., 90., 0., 180., 0.]),
                                     np.array([0., 0., force, 0., 0., 0.])))
        self.assertEqual(len(scraper.records), 2)
        self.assertEqual(scraper.control[0][2], 10.)

    def test_point_decimation_halves_even_samples_already_farther_apart_than_sample_mm(self):
        full, half = PointSampler(.025), PointSampler(.025, 2)
        points = [np.array([x, 0., 100.]) for x in np.arange(0., 1.2, .1)]
        self.assertEqual(sum(full.keep(point) for point in points), 12)
        self.assertEqual(sum(half.keep(point) for point in points), 6)
        self.assertEqual(half.candidates, full.candidates)

    def test_missing_measurement_does_not_stop_a_planned_stroke(self):
        def unavailable():
            raise TimeoutError('delayed measurement')
        scraper = StageScraper.__new__(StageScraper)
        scraper.transport = SimpleNamespace(state=unavailable)
        scraper.records, scraper.missed_samples = [], 0
        self.assertFalse(scraper.measure(AXIS)(np.zeros(6), np.zeros(6)))
        self.assertEqual(scraper.missed_samples, 1)
        self.assertFalse(scraper.records)

    def test_interrupt_saves_subplots_even_if_force_release_fails(self):
        with tempfile.TemporaryDirectory() as directory:
            figure = FigureBook(Path(directory) / 'report.png', 1)
            figure.subplots()[1].plot([0., 1.], [.5, .4])
            runner = DefaultScan.__new__(DefaultScan)
            runner.program = SimpleNamespace(log=lambda text: None, topics=Mock(),
                probe=SimpleNamespace(close=Mock(side_effect=RuntimeError('release failed'))))
            runner.channel, runner.stages = None, []
            runner.save_stage = Mock(return_value=SimpleNamespace(regions=[]))
            scraper = SimpleNamespace(run=Mock(side_effect=KeyboardInterrupt))
            plan = SimpleNamespace(paths=[1], sweeps=[([np.zeros((2, 3))], AXIS, [])])
            try:
                with patch('rokey.part_scan.staged_scan.StageScraper', return_value=scraper):
                    with self.assertRaisesRegex(RuntimeError, 'release failed'):
                        runner.execute(plan, self.scan, 'survey', SimpleNamespace(figures=figure))
                runner.save_stage.assert_called_once()
                self.assertGreater(figure.path.stat().st_size, 1000)
                self.assertFalse(runner.stages[0]['completed'])
            finally:
                figure.close()

    def test_detail_routes_stay_in_roi_and_original_mask(self):
        x, y = np.meshgrid(np.arange(-8., 8.5, .5), np.arange(-8., 8.5, .5))
        grid = HeightMap(np.c_[x.ravel(), y.ravel(), np.full(x.size, 100.)], AXIS, .5, 1., 1.)
        parent = PartPlan.__new__(PartPlan)
        parent.clearance = SimpleNamespace(clear=lambda p, a: True)
        parent.reach, parent.travel = SimpleNamespace(reachable=lambda p, a, *, transfer=False: True), 200.
        parent.grids, parent.count = [(AXIS, grid)], dict(axes=1, points=1, clear=1, reachable=1)
        valid = grid.inner.copy()
        valid[valid.shape[0] // 2, :] = False  # previously inaccessible stripe must remain excluded
        parent.maps = [(AXIS, grid, k, valid, []) for k in (0, 1)]
        detail = deepcopy(self.scan)
        detail.update(self.settings['detail'])
        detail['min_path_mm'] = detail['measurable']['min_segment_mm'] = 2.
        regions = [dict(axis=AXIS.tolist(), bounds_mm=[-4., -4., 90., 4., 4., 95.])]
        plan = RoiPlan(parent, regions, detail)
        self.assertTrue(plan.paths)
        for points, _ in plan.paths:
            self.assertTrue((np.abs(points[:, :2]) <= 4.).all())
        for _, _, _, mask, _ in plan.maps:
            self.assertFalse((mask & ~valid).any())
        survey = DefaultScan.survey_settings(self.scan)
        self.assertEqual(survey['force_n'], .5)
        self.assertEqual(self.scan['force_n'], 2.)


if __name__ == '__main__':
    unittest.main()
