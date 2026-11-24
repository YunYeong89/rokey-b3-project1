"""Detection depth corrected for the robot's posture wobble, for one fixed press axis (mesh-height: bonnet BASE Z,
door X with its tilted TCP).

The measured ball centre (FK of the measured joints, the controller's pose on DSR) carries the wobble. Across the axis
it moves the contact off the planned point, so the depth is read against the planned ball-centre surface at the
measured centre (interpolated over the path's planned centres), not against the planned point of the same instant:
slope x (lateral + along-track) error leaves the depth. Along the axis the ball stays on the surface and the wobble
shows up as a reaction change against the contact (+ probe and gripper) stiffness: that share, (F - median) / k with
k fitted per path on depth and force detrended alike (k >= 0; a dent's force drop deepens it), is taken out of the
contact samples. Run 114347 (bonnet, upright TCP): depth noise 0.148 -> 0.140 mm on path 1; the tilted door TCP turns
more of the wobble into the axis."""
import numpy as np
from scipy.interpolate import LinearNDInterpolator
from scipy.ndimage import median_filter
from scipy.spatial import QhullError


class PoseHeightFilter:
    WINDOW_MM = 25.  # along-track trend kept out of the stiffness fit (dents are 5-7 mm wide)

    def __init__(self, axis, threshold):
        self.axis = np.asarray(axis, float) / np.linalg.norm(axis)
        u = np.array([1., 0., 0.]) if abs(self.axis[0]) < .9 else np.array([0., 1., 0.])
        u -= self.axis * (u @ self.axis)
        self.u = u / np.linalg.norm(u)
        self.v = np.cross(self.axis, self.u)
        self.threshold = threshold  # N: contact samples only
        self.stiffness = np.inf  # N/mm, the last path's fit

    def plane(self, points):
        return np.c_[points @ self.u, points @ self.v]

    def reference(self, centres, targets):
        """Depth (mm, + deeper) of each measured ball centre below the planned centres' surface at its own place
        across the axis; the planned point of the same instant outside their hull."""
        timed = targets @ self.axis
        try:
            planned = LinearNDInterpolator(self.plane(targets), timed)(self.plane(centres))
        except QhullError:
            # A stopped or straight-line stroke may not cover a two-dimensional patch.
            return timed - centres @ self.axis
        return np.where(np.isfinite(planned), planned, timed) - centres @ self.axis

    def detrend(self, values, centres):
        travel = np.r_[0., np.cumsum(np.linalg.norm(np.diff(centres, axis=0), axis=1))]
        grid = np.arange(0., travel[-1] + .1, .1)
        first = np.unique(travel, return_index=True)[1]
        trend = median_filter(np.interp(grid, travel[first], values[first]), int(self.WINDOW_MM / .1) | 1,
                              mode='nearest')
        return values - np.interp(travel, grid, trend)

    def compliance(self, depth, force, centres):
        """`depth` less the reaction's share along the axis on contact samples; self.stiffness the fitted k."""
        contact = (force > self.threshold) & np.isfinite(depth)
        if contact.sum() < 50:
            return depth
        d = self.detrend(depth[contact], centres[contact])
        f = self.detrend(force[contact], centres[contact])
        slope = max(float(d @ f) / float(f @ f), 0.)  # mm/N
        self.stiffness = 1 / slope if slope else np.inf
        corrected = depth.copy()
        corrected[contact] -= slope * (force[contact] - np.median(force[contact]))
        return corrected
