"""Summarize saved DRFL emulator measurements without third-party packages."""
import csv
import json
from pathlib import Path


class BenchmarkSummary:
    def __init__(self, prefix):
        self.prefix = prefix

    @staticmethod
    def distribution(values):
        values = sorted(values)
        if not values:
            return None
        def percentile(p):
            i = (len(values) - 1) * p
            lo = int(i)
            hi = min(lo + 1, len(values) - 1)
            return values[lo] + (values[hi] - values[lo]) * (i - lo)
        return {'count': len(values), 'mean_ms': sum(values) / len(values),
                'p50_ms': percentile(.5), 'p99_ms': percentile(.99),
                'max_ms': values[-1]}

    def rows(self, suffix):
        with Path(str(self.prefix) + suffix).open() as file:
            return list(csv.DictReader(file))

    def summarize(self):
        samples = self.rows('.csv')
        requests = self.rows('_requests.csv')
        monitoring = self.rows('_monitoring.csv')
        result = json.loads(Path(str(self.prefix) + '_motion.json').read_text())
        for key in ['interval_ms', 'lateness_ms', 'pose_call_ms']:
            rows = samples[1:] if key == 'interval_ms' else samples
            result[key] = self.distribution([float(r[key]) for r in rows])
        result['version_request_ms'] = self.distribution([float(r['request_ms']) for r in requests])
        result['monitoring_interval_ms'] = self.distribution([float(r['interval_ms']) for r in monitoring])
        return result


if __name__ == '__main__':
    root = Path(__file__).resolve().parent / 'results'
    summaries = {path.name.removesuffix('_motion.json'):
                 BenchmarkSummary(Path(str(path).removesuffix('_motion.json'))).summarize()
                 for path in sorted(root.glob('*_motion.json'))}
    (root / 'summary.json').write_text(json.dumps(summaries, indent=2) + '\n')
    print(json.dumps(summaries, indent=2))
