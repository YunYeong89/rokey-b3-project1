"""One timestamped measurement, shared by in-memory and process-backed recorders."""
import numpy as np


class PointSpacing:
    """defect_scan's rule: accept an actual point only after moving spacing from the last accepted point."""
    def __init__(self, spacing):
        self.spacing, self.last = spacing, None

    def accept(self, point):
        position = np.asarray(point[:3])
        if self.last is not None and np.linalg.norm(position - self.last) < self.spacing:
            return False
        self.last = position
        return True


def measurement(settings, path, elapsed, controller, state, dt):
    c = controller
    point = state.centre(settings.radius) - settings.radius * c.normal
    alignment = np.arccos(np.clip(-state.rotation[:, 2] @ c.normal, -1, 1))
    valid = (c.phase == 'TRACK' and c.risk < .5 and abs(c.force - settings.force) < .35
             and alignment < np.deg2rad(6))
    row = dict(time_s=elapsed, phase=c.phase, source_sequence=state.sequence,
        source_time_s=state.sampled_at, dt_s=dt, x_m=state.position[0], y_m=state.position[1], z_m=state.position[2],
        fx_n=state.wrench[0], fy_n=state.wrench[1], fz_n=state.wrench[2], fn_n=c.force,
        raw_magnitude_n=float(np.linalg.norm(state.fast_wrench[:3])),
        fast_magnitude_n=c.fast, predicted_n=c.predicted, risk=c.risk, target_n=c.goal,
        feed_mps=c.feed, progress_m=path.progress, command_progress_m=c.command_progress,
        alignment_rad=alignment, moment_residual_nm=c.residual, moment_filtered_nm=c.moment_level,
        tangential_n=c.ft, edge=bool(c.edging),
        governor=c.governor.scale, slope=c.governor.hold,
        normal_x=c.normal[0], normal_y=c.normal[1], normal_z=c.normal[2],
        **{key: state.control.get(key) for key in ('force_feedback_n', 'force_target_n', 'normal_velocity_mps',
           'normal_command_mps', 'normal_reference_error_m', 'normal_reference_saturated',
           'angular_tracking_error_rad', 'joint_tracking_error_rad', 'raw_peak_n',
           'force_innovation_n', 'innovation_sigma_n', 'force_rate_nps')}, valid=bool(valid))
    return row, [*point, c.force]
