"""Live external-force graph: the controller's wrench with the TCP x, y, z over time (virtual or real). The
container has no display, so rokey/images/force_live.png is redrawn every second (keep it open in VS Code; --no-png:
not, the web chart draws the stream instead); the samples also go to /robot/force_graph for the web robot tab, which
draws the same four panels in the browser. On exit a full graph and a CSV are saved (not with --no-save: the web's
monitor). Runs alongside any other robot program.
  ros2 run rokey draw_graph [--rate 30] [--window 30] [--compensated] [--duration 60] [--no-png] [--no-save]
"""
import argparse

if __package__:
    from .robot_namespace import program_args, robot_topic
else:
    from robot_namespace import program_args, robot_topic
import csv
from datetime import datetime
import json
import time

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from std_msgs.msg import String

if __package__:
    from .force_data import CALIBRATION_DIR, IMAGE_DIR, CalibrationRun, ForceData
    from .force_physics import PhysicsCompensator
    from .rotations import rotation_zyz
else:
    from force_data import CALIBRATION_DIR, IMAGE_DIR, CalibrationRun, ForceData
    from force_physics import PhysicsCompensator
    from rotations import rotation_zyz


class ForceGraph:
    def __init__(self, run, threshold, model=None):
        self.run, self.threshold, self.model = run, threshold, model

    def sample(self):
        """One sample; skipped while the controller has no reading (the simulation pauses for the probe handoff)."""
        try:
            self.run.recorder.collect(*self.run.probe.read())
        except (RuntimeError, TimeoutError):
            pass

    def data(self, window=None):
        rows = np.asarray(self.run.recorder.rows)
        if window:
            rows = rows[rows[:, 0] >= rows[-1, 0] - window]
        return ForceData(rows, [''] * len(rows))

    @staticmethod
    def to_base(data, wrench):
        rotations = np.array([rotation_zyz(pose[3:]) for pose in data.pose])
        return np.einsum('nij,nj->ni', rotations, wrench[:, :3])

    def draw(self, path, window=None):
        data = self.data(window)
        t = data.t - self.run.recorder.rows[0][0]
        force = self.to_base(data, data.wrench)
        fig, axes = plt.subplots(2, 2, figsize=(15, 9))
        for index, (axis, color) in enumerate(zip('xyz', ['tab:red', 'tab:green', 'tab:blue'])):
            axes[0, 0].plot(t, force[:, index], color=color, lw=0.9, label=f'F{axis}')
        axes[0, 1].plot(t, np.linalg.norm(force, axis=1), color='0.3', lw=0.9, label='|F|')
        axes[0, 1].plot(t, ForceData.normal(data.wrench), color='tab:purple', lw=0.9, label='Fn (TOOL -Z)')
        if self.model is not None:
            corrected = self.model.correct(data)
            for index, color in enumerate(['tab:red', 'tab:green', 'tab:blue']):
                axes[0, 0].plot(t, self.to_base(data, corrected)[:, index], color=color, lw=0.8, ls='--')
            axes[0, 1].plot(t, ForceData.normal(corrected), color='tab:purple', lw=0.8, ls='--', label='Fn calibrated')
        axes[0, 1].axhline(self.threshold, color='tab:orange', ls=':', label=f'touch threshold {self.threshold:.2f} N')
        for index, (axis, color) in enumerate(zip('xyz', ['tab:red', 'tab:green', 'tab:blue'])):
            axes[1, 0].plot(t, data.pose[:, index], color=color, lw=0.9, label=f'TCP {axis}')
        path_plot = axes[1, 1].scatter(data.pose[:, 0], data.pose[:, 1], c=np.linalg.norm(force, axis=1),
                                       s=6, cmap='viridis')
        fig.colorbar(path_plot, ax=axes[1, 1], label='|F| [N]')
        titles = [('BASE external force' + (' (dashed: calibrated)' if self.model else ''), 'F [N]'),
                  ('Force magnitude and contact force', 'F [N]'), ('TCP position (BASE)', 'mm'), (None, 'y [mm]')]
        for axis, (title, label) in zip(axes.flat[:3], titles[:3]):
            axis.set(title=title, xlabel='time [s]', ylabel=label)
            axis.legend(fontsize=8, loc='upper left')
        axes[1, 1].set(title='TCP path coloured by |F|', xlabel='x [mm]', ylabel='y [mm]', aspect='equal')
        for axis in axes.flat:
            axis.grid(alpha=0.3)
        fig.suptitle(f'External force ({self.run.mode}, tool={self.run.meta["tool"]}, tcp={self.run.meta["tcp"]}, '
                     f'{len(t)} samples{f", last {window:.0f} s" if window else ""})')
        fig.tight_layout()
        fig.savefig(path, dpi=110)
        plt.close(fig)

    def save_csv(self, path):
        data = self.data()
        force = self.to_base(data, data.wrench)
        with path.open('w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['t_s', 'x_mm', 'y_mm', 'z_mm', 'a_deg', 'b_deg', 'c_deg', 'Fx_N', 'Fy_N', 'Fz_N', 'Fn_N'])
            writer.writerows(np.c_[data.t - data.t[0], data.pose, force, ForceData.normal(data.wrench)].round(5))


class GraphStream:
    """The live graph's samples for the web chart: /robot/force_graph (std_msgs/String JSON), a batch about every
    0.1 s with the samples since the last one: t (s from the first sample), force (BASE) and fn (N), position (TCP x y z,
    mm), with --compensated force_calibrated and fn_calibrated too; plus mode, tool, tcp (names) and threshold."""
    CONTEXT = 40  # rows before the new ones: the calibrated model differentiates the pose

    def __init__(self, graph):
        run = graph.run
        self.graph, self.sent = graph, 0
        self.publisher = run.node.create_publisher(String, robot_topic(run.node, '/robot/force_graph'), 10)
        self.meta = dict(mode=run.mode, tool=run.meta['tool'], tcp=run.meta['tcp'], threshold=graph.threshold)

    def publish(self):
        rows = self.graph.run.recorder.rows
        new = len(rows) - self.sent
        if new <= 0 or len(rows) < 3:
            return
        data = ForceData(rows[max(0, self.sent - self.CONTEXT):], [''] * (len(rows) - max(0, self.sent - self.CONTEXT)))
        tail = slice(-new, None)
        batch = dict(self.meta, t=(data.t[tail] - rows[0][0]).round(4).tolist(),
                     force=self.graph.to_base(data, data.wrench)[tail].round(3).tolist(),
                     fn=ForceData.normal(data.wrench)[tail].round(3).tolist(),
                     position=data.pose[tail, :3].round(2).tolist())
        if self.graph.model is not None:
            corrected = self.graph.model.correct(data)
            batch.update(force_calibrated=self.graph.to_base(data, corrected)[tail].round(3).tolist(),
                         fn_calibrated=ForceData.normal(corrected)[tail].round(3).tolist())
        self.publisher.publish(String(data=json.dumps(batch)))
        self.sent = len(rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--rate', type=float, default=30.0, help='sampling rate limit, Hz (default: 30)')
    parser.add_argument('--window', type=float, default=30.0, help='live graph span, s (default: 30)')
    parser.add_argument('--compensated', action='store_true', help='overlay the force_physics calibration')
    parser.add_argument('--duration', type=float, help='stop after this many seconds (default: Ctrl+C)')
    parser.add_argument('--no-png', action='store_true', help='no live PNG (the web chart draws the stream)')
    parser.add_argument('--no-save', action='store_true', help='no full graph / CSV on exit (the web monitor)')
    options = parser.parse_args(program_args())
    run = CalibrationRun('draw_graph')
    model = PhysicsCompensator.load(CALIBRATION_DIR / f'force_physics_{run.mode}.json') if options.compensated else None
    graph = ForceGraph(run, run.config['scan']['touch_threshold'], model)
    stream = GraphStream(graph)
    IMAGE_DIR.mkdir(exist_ok=True)
    live, start, redraw = IMAGE_DIR / 'force_live.png', time.monotonic(), time.monotonic() + 1.0
    published = start
    run.log('streaming /robot/force_graph' + ('' if options.no_png else f', drawing {live} every second')
            + '; Ctrl+C to finish')
    try:
        while not options.duration or time.monotonic() - start < options.duration:
            tick = time.monotonic()
            graph.sample()
            if tick >= published + 0.1:
                stream.publish()
                published = tick
            if not options.no_png and tick >= redraw and len(run.recorder.rows) > 1:
                graph.draw(live, options.window)
                redraw = tick + 1.0
            time.sleep(max(0.0, 1 / options.rate - (time.monotonic() - tick)))
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if not options.no_save and len(run.recorder.rows) > 1:
                stem = IMAGE_DIR / f'force_graph_{datetime.now():%Y%m%d_%H%M%S}'
                graph.draw(stem.with_suffix('.png'))
                graph.save_csv(stem.with_suffix('.csv'))
                run.log(f'saved {stem}.png / .csv ({len(run.recorder.rows)} samples)')
        finally:
            run.close()


if __name__ == '__main__':
    main()
