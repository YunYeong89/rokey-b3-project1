"""One experiment -> CSV, measured points, model metadata and a compact summary."""
import csv
from dataclasses import asdict
from datetime import datetime
import json
from math import sqrt
from pathlib import Path

import pymunk


class Recording:
    @staticmethod
    def summary(experiment):
        rows = experiment.rows
        track = [row for row in rows if row['phase'] == 'TRACK']
        error = [row['fn_n'] - row['target_n'] for row in track]
        return dict(phase=experiment.controller.phase, reason=experiment.controller.reason,
                    simulated_seconds=round(experiment.time, 3), samples=len(rows), tracking_samples=len(track),
                    points=len(experiment.points),
                    travelled_x_mm=round((experiment.physics.body.position.x - experiment.start_x) * 1000, 3),
                    force_rmse_n=sqrt(sum(e * e for e in error) / len(error)) if error else None,
                    force_in_band_fraction=sum(abs(e) <= .35 for e in error) / len(error) if error else None,
                    physical_peak_contact_n=experiment.physical_peak,
                    physics_step_peak_contact_n=experiment.physics_peak,
                    local_measured_peak_n=experiment.local_sensor_peak,
                    stop_confirmed=experiment.stop_confirmed, local_stop_confirmed=experiment.local_stop_confirmed,
                    unload_events=experiment.controller.unload_events,
                    scheduler_counts=experiment.counts, feedback_received=experiment.feedback_received,
                    mean_control_dt_ms=(1000 * sum(experiment.control_intervals) / len(experiment.control_intervals)
                                        if experiment.control_intervals else None),
                    max_state_age_ms=max((r['state_age_ms'] for r in rows), default=0), events=experiment.events,
                    peak_raw_contact_n=max(((r['raw_fx_n'] ** 2 + r['raw_fy_n'] ** 2) ** .5 for r in rows), default=0),
                    mean_normal_error_deg=sum(r['normal_error_deg'] for r in track) / len(track) if track else None,
                    physics='Pymunk ' + pymunk.version, settings=asdict(experiment.settings),
                    initial_settings=experiment.initial_settings, parameter_changes=experiment.parameter_changes,
                    sensor_model='Preset-defined contact-wrench sampling, rotating sensor bias / optional drift / inertial residual + noise, free-space tare, delayed state feedback, separate raw / filtered channels.',
                    actuator_model='Cartesian surrogate with finite stiffness, command delay / hold / expiry, rate and acceleration limits, actual tool-axis pressing, finite actuator response. Not identified M0609 dynamics.',
                    normal_source='Force + measured-center history, no terrain/collision normals supplied to controller.')

    @classmethod
    def save(cls, experiment, directory=None):
        if directory is None:
            name = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
            directory = Path(__file__).resolve().parents[1] / 'results' / f'{name}_{experiment.settings.controller}_{experiment.settings.scene}'
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        if experiment.rows:
            with (directory / 'signals.csv').open('w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=list(experiment.rows[0]))
                writer.writeheader()
                writer.writerows(experiment.rows)
        with (directory / 'points.csv').open('w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['x_m', 'y_m'])
            writer.writerows(experiment.points)
        summary = cls.summary(experiment)
        (directory / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
        return directory
