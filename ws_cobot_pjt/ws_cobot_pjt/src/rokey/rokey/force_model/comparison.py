"""Compare service and RT sampling during the same free-space calibration run."""
from dataclasses import dataclass
import json

import numpy as np

from ..configs import load
from ..part_scan.kinematics import RobotChain
from .kalman import ForceKalman
from .model import ForceSamples
from .selection import Selection, label, preserves_axes, stationary


@dataclass
class Trace:
    time: np.ndarray
    phases: np.ndarray
    forces: np.ndarray  # (sample, processing stage, BASE axis)
    model: str
    kalman_passed: bool


class TelemetryComparison:
    def __init__(self, path, settings):
        self.path, self.settings = path, dict(settings, moment_weight_m=0.)
        with np.load(path) as data:
            if 'rt_rows' not in data or not len(data['rt_rows']):
                raise ValueError(f'{path.name}: --compare-telemetry로 수집한 RT 표본이 없습니다.')
            meta = json.loads(str(data['meta']))
            self.legacy = ForceSamples(data['rows'], data['labels'])
            self.rt = ForceSamples(data['rt_rows'], data['rt_labels'], wrench_in_base=True,
                                   tcp_velocity_mm_s=data['rt_tcp_velocity_mm_s'])
            self.rt_receipt = data['rt_receipt_s']
        self.chain = RobotChain(meta['urdf'], meta['tcp_mm_deg'], load('robot')['real_scan']['ik'])

    def fit_trace(self, samples, receipt, phases):
        train = np.isin(samples.labels, phases) & (samples.station != 'V')
        test = np.isin(samples.labels, phases) & (samples.station == 'V')
        if len(set(samples.station[train])) < 2 or not np.any(test):
            raise ValueError('비교에는 양쪽에 기록된 학습 스테이션 2개 이상과 V 검증 구간이 필요합니다.')
        selection = Selection(samples.subset(train), self.chain, self.settings)
        _, _, best = selection.run()
        validation = samples.subset(test)
        selected, corrected = selection.validate(best, validation)
        raw = validation.wrench[:, :3]
        corrected = corrected[:, :3]
        filtered, _ = ForceKalman.run(self.settings['kalman'], validation.t, corrected,
                                      stationary(validation.labels))
        description = label(selected, ' ')
        if selected is not best:
            description += f' ({label(best, " ")} failed on V)'
        passed = preserves_axes(filtered, corrected, validation.labels)
        return Trace(receipt[test], validation.labels, np.stack([raw, corrected, filtered], axis=1),
                     description, passed)

    @staticmethod
    def aligned(legacy, rt):
        """Use a shared receive-time grid within matching phases, keeping the last observed sample."""
        times, masks, pairs = [], [], []
        for phase in dict.fromkeys(legacy.phases):
            a, b = legacy.phases == phase, rt.phases == phase
            if not np.any(b):
                continue
            ta, tb = legacy.time[a], rt.time[b]
            grid = np.arange(max(ta[0], tb[0]), min(ta[-1], tb[-1]), .01)
            ia = np.searchsorted(ta, grid, side='right') - 1
            ib = np.searchsorted(tb, grid, side='right') - 1
            keep = (grid - ta[ia] <= .1) & (grid - tb[ib] <= .1)
            if not np.any(keep):
                continue
            times.append(np.r_[grid, grid[-1]])
            masks.append(np.r_[np.repeat(stationary([phase])[0], len(grid)), False])
            paired = np.stack([legacy.forces[a][ia], rt.forces[b][ib]], axis=1)
            paired[~keep] = np.nan
            pairs.append(np.concatenate([paired, np.full((1, 2, 3, 3), np.nan)]))
        if not times:
            raise ValueError('두 입력이 같은 시각에 기록한 V 검증 구간이 없습니다.')
        time = np.concatenate(times)
        return time - time[0], np.concatenate(masks), np.concatenate(pairs)

    def save(self):
        import matplotlib.pyplot as plt

        phases = np.intersect1d(self.legacy.labels, self.rt.labels)
        legacy = self.fit_trace(self.legacy, self.legacy.t, phases)
        rt = self.fit_trace(self.rt, self.rt_receipt, phases)
        time, still, values = self.aligned(legacy, rt)
        fig, panels = plt.subplots(3, 2, sharex=True, sharey='row', figsize=(15, 9))
        fig.suptitle('Same run, held-out V station (free space; shaded: stationary)')
        for source, (headline, trace) in enumerate(zip(('Without RT topic', 'With RT topic'), (legacy, rt))):
            kalman_status = 'passed' if trace.kalman_passed else 'failed axis checks'
            panels[0, source].set_title(f'{headline}\n{trace.model} | Kalman {kalman_status}')
            for axis, name in enumerate('xyz'):
                panel = panels[axis, source]
                panel.fill_between(time, 0, 1, where=still, step='post', color='.92',
                                   transform=panel.get_xaxis_transform())
                panel.axhline(0, color='.3', linewidth=.6)
                for stage, title, color, style in zip(range(3), ('Raw', 'Physics', 'Kalman force'),
                                                      ('.4', 'tab:orange', 'tab:blue'), ('-', '--', '-')):
                    force = values[:, source, stage, axis]
                    valid = force[np.isfinite(force)]
                    rms = np.sqrt(np.mean(valid ** 2))
                    p95 = np.percentile(np.abs(valid), 95)
                    panel.plot(time, force, color=color, linestyle=style, linewidth=.75,
                               label=f'{title} · RMS {rms:.3f} / P95 {p95:.3f} N')
                panel.set_ylabel(f'BASE F{name} (N)')
                panel.legend(fontsize=8, loc='upper right')
                panel.grid(alpha=.2)
            panels[-1, source].set_xlabel('Elapsed receive time (s)')
        path = self.path.with_name(f'{self.path.stem}_telemetry_comparison.png')
        fig.tight_layout()
        fig.savefig(path, dpi=120)
        plt.close(fig)
        return path
