"""Time-series recording and plots for force_slide; BASE pose and raw TOOL wrench."""
import csv
from datetime import datetime
from pathlib import Path
import threading
import time

import numpy as np


class ForceSlideRecorder:
    image_dir = Path(__file__).resolve().parents[2] / 'image'
    period_s = 0.1  # Up to 10 Hz; pose and wrench are queried sequentially.

    def __init__(self, robot, force_n):
        self.robot = robot
        self.force_n = force_n
        self.phase = 'start'
        self.rows = []
        self.stopped = threading.Event()
        self.thread = None

    def start(self):
        self.started = time.monotonic()
        self.thread = threading.Thread(target=self.collect, name='force_slide_recorder', daemon=True)
        self.thread.start()

    def collect(self):
        while not self.stopped.is_set():
            tick, phase = time.monotonic(), self.phase
            try:
                # RobotServices serializes service calls and executor access with its lock.
                pose, _ = self.robot.get_current_posx(ref=self.robot.DR_BASE)
                wrench = self.robot.get_tool_force(ref=self.robot.DR_TOOL)
            except (RuntimeError, TimeoutError) as error:
                self.robot.node.get_logger().warning(f'Sensor recording stopped: {error}')
                break
            self.rows.append([time.monotonic() - self.started, phase, *pose, *wrench])
            self.stopped.wait(max(0.0, self.period_s - (time.monotonic() - tick)))

    def stop(self):
        self.stopped.set()
        if self.thread is not None:
            self.thread.join()

    def save(self):
        """Call after stop(); keep the original values in a CSV beside the plot."""
        if not self.rows:
            return None
        import matplotlib

        matplotlib.use('Agg')
        import matplotlib.pyplot as plt

        self.image_dir.mkdir(parents=True, exist_ok=True)
        stem = self.image_dir / f'force_slide_{datetime.now():%Y%m%d_%H%M%S_%f}'
        with stem.with_suffix('.csv').open('w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['time_s', 'phase', 'base_x_mm', 'base_y_mm', 'base_z_mm',
                             'base_a_deg', 'base_b_deg', 'base_c_deg',
                             'tool_fx_n', 'tool_fy_n', 'tool_fz_n',
                             'tool_mx_nm', 'tool_my_nm', 'tool_mz_nm'])
            writer.writerows(self.rows)

        data = np.asarray([[row[0], *row[2:]] for row in self.rows], dtype=float)
        fig, axes = plt.subplots(2, 2, figsize=(15, 9), sharex=True)
        plots = [
            ('Measured force (TOOL)', 'Force [N]', 7, ('Fx', 'Fy', 'Fz')),
            ('Measured torque (TOOL)', 'Torque [N m]', 10, ('Mx', 'My', 'Mz')),
            ('TCP position (BASE)', 'Position [mm]', 1, ('X', 'Y', 'Z')),
            ('TCP orientation (BASE, ZYZ)', 'Angle [deg]', 4, ('A', 'B', 'C')),
        ]
        for axis, (title, unit, start, labels) in zip(axes.flat, plots):
            for offset, label in enumerate(labels):
                axis.plot(data[:, 0], data[:, start + offset], label=label, linewidth=1)
            axis.set(title=title, xlabel='Time [s]', ylabel=unit)
            axis.grid(alpha=0.3)
            axis.legend(loc='upper left')

        previous = None
        for row in self.rows:
            if row[1] != previous:
                for axis in axes.flat:
                    axis.axvline(row[0], color='gray', linestyle=':', alpha=0.4)
                axes[0, 0].text(row[0], 1.01, row[1], rotation=45, fontsize=7,
                                transform=axes[0, 0].get_xaxis_transform())
                previous = row[1]
        fig.suptitle(f'Force slide: commanded TOOL Fz={self.force_n:+.2f} N; '
                     'measured TOOL wrench and BASE TCP pose', y=1.02)
        fig.tight_layout()
        path = stem.with_suffix('.png')
        try:
            fig.savefig(path, dpi=150, bbox_inches='tight')
        finally:
            plt.close(fig)
        return path
