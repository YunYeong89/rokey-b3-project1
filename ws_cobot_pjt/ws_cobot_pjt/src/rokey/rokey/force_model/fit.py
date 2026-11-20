"""Identification of the joint-space force model from force_model_collect data.
Candidates must match or improve raw RMS and P95 on every BASE force axis in leave-one-station-out validation.
The selected model is refitted on the training stations and checked on V if the recording includes it.
The result is saved to calibration/force_model_<mode>.json with
  force_model_<mode>.png       the scores, then raw vs corrected Fz per station (its held-out prediction)
  force_model_<mode>_axes.png  raw / physics model / model + Kalman layer per BASE axis (force_model/view.py): in the
                               air the corrected curves should stay in the noise band around 0
  ros2 run rokey force_model_fit [FILES ...]      (default: every calibration/force_model_<mode>_*.npz, --mode real)"""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from ..configs import CALIBRATION_DIR, load
from ..part_scan.kinematics import RobotChain
from .model import ForceSamples
from .selection import Selection, label, rms, stationary
from .view import axes_figure, report


class ModelSelection(Selection):
    def __init__(self, files, settings):
        rows, labels, metas = [], [], []
        for path in files:
            with np.load(path) as data:
                rows.append(data['rows'])
                labels.append(data['labels'])
                metas.append(json.loads(str(data['meta'])))
        self.meta = metas[-1]
        chain = RobotChain(self.meta['urdf'], self.meta['tcp_mm_deg'], load('robot')['real_scan']['ik'])
        samples = ForceSamples(np.concatenate(rows), np.concatenate(labels))
        validation = samples.station == 'V'
        self.validation = samples.subset(validation) if np.any(validation) and np.any(~validation) else None
        super().__init__(samples if self.validation is None else samples.subset(~validation), chain, settings)

    def save(self, raw, results, best):
        mode = self.meta['mode']
        candidate = best
        if self.validation is not None:
            best, _ = self.validate(candidate, self.validation)
            if best is not candidate:
                print(f'{label(candidate, " ")}: V axis checks failed. Selected raw.')
        scores = [{key: result[key] for key in ('name', 'epsilon', 'memory', 'force_rms', 'fz_rms', 'accepted')}
                  for result in [raw, *results]]
        path = CALIBRATION_DIR / f'force_model_{mode}.json'
        best['model'].save(path, {key: self.meta[key] for key in ('mode', 'tool', 'tcp', 'tcp_mm_deg', 'created')} | dict(
            model=best['name'], terms=list(best['groups']), held_out=scores, samples=len(self.samples.t),
            stations=self.stations, compliance=self.meta['compliance'],
            validation_station='V' if self.validation is not None else None, candidate=label(candidate, ' ')))
        self.figure(raw, results, best, CALIBRATION_DIR / f'force_model_{mode}.png')
        still = stationary(self.samples.labels)
        threshold = load('robot')['real_scan']['mesh_height']['contact_threshold_n']
        stages = report(self.samples, best, self.settings['kalman'], still, threshold)
        axes_figure(CALIBRATION_DIR / f'force_model_{mode}_axes.png', self.samples.t, stages, still,
                    self.settings['kalman']['noise_n'], threshold,
                    f'force_model_collect in the air: physics model {label(best, ", ")} (held out) + Kalman')
        return path, scores

    def figure(self, raw, results, best, path):
        columns = 2
        rows = 1 + int(np.ceil(len(self.stations) / columns))
        fig = plt.figure(figsize=(15, 3.2 * rows))
        panel = fig.add_subplot(rows, 1, 1)
        names = [label(r) for r in [raw, *results]]
        x = np.arange(len(names))
        panel.bar(x - .2, [r['force_rms'] for r in [raw, *results]], .4, label='|F| RMS')
        panel.bar(x + .2, [r['fz_rms'] for r in [raw, *results]], .4, label='Fz RMS')
        panel.set_xticks(x, names, fontsize=7)
        panel.set(ylabel='N', title=f'Leave-one-station-out residual (best: {label(best, " ")})')
        panel.legend()
        for index, station in enumerate(self.stations):
            panel = fig.add_subplot(rows, columns, columns + index + 1)
            k = self.samples.station == station
            t = self.samples.t[k] - self.samples.t[k][0]
            panel.plot(t, raw['corrected'][k, 2], color='.5', linewidth=.4, label='raw')
            panel.plot(t, best['corrected'][k, 2], color='tab:blue', linewidth=.4, label='corrected (held out)')
            panel.set(title=f'{station}: Fz RMS {rms(raw["corrected"][k, 2]):.3f} -> {rms(best["corrected"][k, 2]):.3f} N',
                      xlabel='s', ylabel='Fz (N)')
            panel.legend(fontsize=7)
        fig.tight_layout()
        fig.savefig(path, dpi=120)
        plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('files', nargs='*')
    parser.add_argument('--mode', default='real')
    parser.add_argument('--compare-telemetry', action='store_true',
                        help='plot matched service/RT data without saving a production calibration model')
    options = parser.parse_args()
    files = options.files or sorted(str(path) for path in CALIBRATION_DIR.glob(f'force_model_{options.mode}_*.npz'))
    if not files:
        parser.error(f'{CALIBRATION_DIR}/force_model_{options.mode}_*.npz 가 없습니다: ros2 run rokey force_model_collect')
    settings = load('robot')['force_model']
    if options.compare_telemetry:
        from .comparison import TelemetryComparison
        if not options.files:
            for path in reversed(files):
                with np.load(path) as data:
                    if 'rt_rows' in data and len(data['rt_rows']):
                        files = [path]
                        break
            else:
                parser.error('force_model_collect --cell-clear --compare-telemetry 로 두 입력을 함께 수집하세요.')
        for path in files:
            try:
                print(f'-> {TelemetryComparison(Path(path), settings).save()}')
            except ValueError as error:
                parser.error(str(error))
        return
    selection = ModelSelection(files, settings)
    path, scores = selection.save(*selection.run())
    for score in scores:
        status = 'passed' if score['accepted'] else 'rejected'
        print(f'{label(score, " "):40s} |F| RMS {score["force_rms"]:.3f} N, Fz RMS {score["fz_rms"]:.3f} N  {status}')
    print(f'-> {path} (+ .png, _axes.png)')


if __name__ == '__main__':
    main()
