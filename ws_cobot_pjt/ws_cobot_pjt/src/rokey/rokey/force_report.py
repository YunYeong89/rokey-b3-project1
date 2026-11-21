"""Stability plot of the force reading while the robot moves in free space: raw vs calibrated."""
from datetime import datetime

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

if __package__:
    from .force_data import IMAGE_DIR, ForceData
else:
    from force_data import IMAGE_DIR, ForceData


def summary(values):
    return (f'RMS {np.sqrt(np.mean(values ** 2)):.3f} N, peak {np.abs(values).max():.3f} N, '
            f'std {values.std():.3f} N')


def save_stability_plot(data, corrected, title, name):
    """Free space means the true external force is zero, so every deviation is sensor/model error."""
    t = data.t - data.t[0]
    raw_fn, fn = ForceData.normal(data.wrench), ForceData.normal(corrected)
    fig, axes = plt.subplots(3, 1, figsize=(12, 11))

    axes[0].plot(t, raw_fn, color='0.6', lw=0.8, label='raw: ' + summary(raw_fn))
    axes[0].plot(t, fn, color='tab:blue', lw=0.9, label='calibrated: ' + summary(fn))
    changes = np.flatnonzero(data.labels[1:] != data.labels[:-1]) + 1
    for index in np.r_[0, changes]:
        axes[0].axvline(t[index], color='0.85', lw=0.6)
        axes[0].text(t[index], axes[0].get_ylim()[1], data.labels[index], fontsize=7, rotation=90, va='top')
    axes[0].set(title='Fn (TOOL -Z) during free-space motion', xlabel='time [s]', ylabel='Fn [N]')

    for index, (axis, color) in enumerate(zip('xyz', ['tab:red', 'tab:green', 'tab:blue'])):
        axes[1].plot(t, data.wrench[:, index], color=color, ls='--', lw=0.7, alpha=0.6)
        axes[1].plot(t, corrected[:, index], color=color, lw=0.9, label=f'F{axis}')
    axes[1].set(title='TOOL force: raw (dashed) vs calibrated (solid)', xlabel='time [s]', ylabel='F [N]')

    bins = np.linspace(min(raw_fn.min(), fn.min()), max(raw_fn.max(), fn.max()), 60)
    axes[2].hist(raw_fn, bins, color='0.6', alpha=0.6, label='raw')
    axes[2].hist(fn, bins, color='tab:blue', alpha=0.6, label='calibrated')
    axes[2].set(title='Fn distribution', xlabel='Fn [N]', ylabel='samples')

    for axis in axes:
        axis.grid(alpha=0.3)
        axis.legend(fontsize=8, loc='best')
    fig.suptitle(title)
    fig.tight_layout()
    IMAGE_DIR.mkdir(exist_ok=True)
    path = IMAGE_DIR / f'{name}_{datetime.now():%Y%m%d_%H%M%S}.png'
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path
