"""Small reproducible review on the existing mixed section, through x=460 mm.

Explicitly run with .venv/bin/python review_sensitivity.py. It does not run at GUI startup.
Only adaptive slowing/unloading is disabled in the baseline; hard stop/watchdog stay enabled.
"""
import json
from pathlib import Path

from surface_lab.config import Settings
from surface_lab.recording import Recording
from surface_lab.simulation import Experiment


def main():
    root = Path(__file__).resolve().parent / 'results' / 'robot_sensitivity'
    rows = []
    for method in ('hybrid', 'admittance'):
        for case in ('nominal', 'delayed', 'delayed_baseline'):
            s = Settings(controller=method, feed=.02)
            if case != 'nominal':
                s.feedback_delay, s.command_delay, s.jitter = .06, .03, .01
                s.dropout, s.sensor_tau = .03, .12
            s.adaptive = case != 'delayed_baseline'
            experiment = Experiment(s)
            while experiment.time < 350 and not experiment.stopped:
                experiment.advance(.25)
                if experiment.latest.position.x >= .46 and experiment.brake_started is None:
                    experiment.request_stop('Sensitivity review ROI x=460 mm reached.', 'FINISHED')
            result = Recording.summary(experiment)
            Recording.save(experiment, root / f'{method}_{case}')
            row = dict(method=method, case=case, phase=result['phase'], reason=result['reason'],
                       time_s=result['simulated_seconds'], travelled_x_mm=result['travelled_x_mm'],
                       sensor_interval_true_peak_n=result['physical_peak_contact_n'],
                       physics_step_true_peak_n=result['physics_step_peak_contact_n'],
                       measured_force_rmse_n=result['force_rmse_n'], unloads=result['unload_events'],
                       stop_confirmed=result['stop_confirmed'], max_state_age_ms=result['max_state_age_ms'])
            rows.append(row)
            print(json.dumps(row), flush=True)
    (root / 'comparison.json').write_text(json.dumps({'roi_end_x_m': .46, 'cases': rows}, indent=2) + '\n')


if __name__ == '__main__':
    main()
