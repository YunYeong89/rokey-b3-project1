"""Local 2D probe lab. ./run.sh; H/A controller, 1..4 profile, Space pause, R reset, S save."""
import argparse
import json
from pathlib import Path

from surface_lab.config import Settings
from surface_lab.geometry import Terrain
from surface_lab.recording import Recording
from surface_lab.simulation import Experiment


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--preset', choices=['robot', 'ideal'], default='robot', help='Robot-like sampled I/O, or a low-delay comparison model')
    parser.add_argument('--controller', choices=['hybrid', 'admittance'], default='hybrid')
    parser.add_argument('--scene', choices=Terrain.names, default='mixed')
    parser.add_argument('--speed-mm-s', type=float, default=5.0)
    parser.add_argument('--force', type=float, default=1.5, help='Target normal reaction [N]')
    parser.add_argument('--friction', type=float, default=.15, help='Physical Coulomb coefficient')
    parser.add_argument('--noise', type=float, help='Sensor standard deviation [N]; preset default when omitted')
    parser.add_argument('--time-scale', type=float, help='Playback speed; all simulated I/O clocks scale together')
    parser.add_argument('--feedback-delay-ms', type=float, help='One-way feedback delay [ms]')
    parser.add_argument('--command-delay-ms', type=float, help='Command transport delay [ms]')
    parser.add_argument('--jitter-ms', type=float, help='Bounded transport jitter [ms]')
    parser.add_argument('--dropout', type=float, help='Missed feedback update probability [0,1)')
    parser.add_argument('--sensor-tau-ms', type=float, help='Force low-pass time constant [ms]')
    parser.add_argument('--effective-mass', type=float, help='Assumed Cartesian plant mass [kg], not probe payload mass')
    parser.add_argument('--bias-y', type=float, help='Sensor-frame Y bias before tare [N]')
    parser.add_argument('--drift-y', type=float, help='Residual Y bias drift [N/s]')
    parser.add_argument('--inertia-error', type=float, help='Residual inertial compensation error [kg]')
    parser.add_argument('--no-adaptive', action='store_true', help='Compare without adaptive slowing/unloading; hard stop and watchdog remain on')
    parser.add_argument('--headless', action='store_true', help='No window; default duration is 20 simulated seconds')
    parser.add_argument('--duration', type=float, help='Stop after this many simulated seconds')
    parser.add_argument('--output', type=Path, help='Save CSV + summary to this directory')
    parser.add_argument('--screenshot', type=Path, help='Save the final GUI frame as PNG')
    parser.add_argument('--paused', action='store_true')
    args = parser.parse_args()
    if args.duration is not None and args.duration <= 0:
        parser.error('--duration must be positive.')
    settings = Settings.for_preset(args.preset)
    settings.scene, settings.controller = args.scene, args.controller
    settings.feed, settings.target_force, settings.friction = args.speed_mm_s / 1000, args.force, args.friction
    for argument, field, factor in [('noise','noise',1), ('time_scale','time_scale',1),
                                    ('feedback_delay_ms','feedback_delay',.001), ('command_delay_ms','command_delay',.001),
                                    ('jitter_ms','jitter',.001), ('dropout','dropout',1),
                                    ('sensor_tau_ms','sensor_tau',.001), ('effective_mass','mass',1),
                                    ('bias_y','sensor_bias_y',1), ('drift_y','bias_drift_y',1),
                                    ('inertia_error','inertia_error_mass',1)]:
        value = getattr(args, argument)
        if value is not None:
            setattr(settings, field, value * factor)
    settings.adaptive = not args.no_adaptive
    if not (0 < settings.feed <= .05 and .3 <= settings.target_force <= 5 and 0 <= settings.friction <= 1
            and 0 <= settings.noise <= .5 and 0 < settings.time_scale <= 16 and .05 <= settings.mass <= 20
            and 0 <= settings.feedback_delay <= .5 and 0 <= settings.command_delay <= .5
            and 0 <= settings.jitter <= .1 and 0 <= settings.dropout < 1 and 0 <= settings.sensor_tau <= .5
            and abs(settings.sensor_bias_y) <= 5 and abs(settings.bias_drift_y) <= .1
            and abs(settings.inertia_error_mass) <= 1):
        parser.error('Invalid force / speed / sensor / transport parameter; use --help for units.')
    experiment = Experiment(settings)
    if args.headless:
        experiment.advance(args.duration if args.duration is not None else 20.0)
        print(json.dumps(Recording.summary(experiment), indent=2))
    else:
        from surface_lab.viewer import Application
        Application(experiment, paused=args.paused).run(args.duration, args.screenshot)
    if args.output:
        print('Saved:', Recording.save(experiment, args.output))
    return 1 if experiment.controller.phase == 'STOPPED' else 0


if __name__ == '__main__':
    raise SystemExit(main())
