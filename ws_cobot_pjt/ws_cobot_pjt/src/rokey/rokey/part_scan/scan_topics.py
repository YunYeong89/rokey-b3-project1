"""real_scan on the rokey_interfaces topics (BASE frame, SI units), beside the files that stay the record:
  /scan/samples   ContactSampleArray  every recorded sample (TCP and wrench of one tick), in batches every BATCH_S
  /scan/progress  ScanPart.Feedback   stage, path and segment, stroke progress, points, force, remaining time
  /scan/roi       RoiArray            the regions of the scan, confirmed first (latched)
  /scan/result    ScanPart.Result     the end of the run (latched)
web_control's ScanPart action (/scan_part) relays /scan/progress and /scan/result as its feedback and result."""
import time

import numpy as np

from ..robot_namespace import robot_topic
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3, Wrench
from rclpy.qos import DurabilityPolicy, QoSProfile
from rokey_interfaces.action import ScanPart
from rokey_interfaces.msg import ContactSample, ContactSampleArray, RoiArray, RoiRegion
from scipy.spatial.transform import Rotation

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
PHASES = dict(travel=ContactSample.TRAVEL, retract=ContactSample.RELEASE, search=ContactSample.PRESS,
              stroke=ContactSample.STROKE)


def vector(kind, values):
    x, y, z = map(float, values)
    return kind(x=x, y=y, z=z)


class ScanTopics:
    BATCH_S = .1
    # Feedback stages (message constants live on the class only, not on instances)
    PLANNING, MOVING = ScanPart.Feedback.PLANNING, ScanPart.Feedback.MOVING
    SCANNING, REPORTING = ScanPart.Feedback.SCANNING, ScanPart.Feedback.REPORTING

    def __init__(self, node, run_id, part):
        self.node, self.run_id, self.part = node, run_id, part
        self.samples = node.create_publisher(ContactSampleArray, robot_topic(node, '/scan/samples'), 10)
        self.progress = node.create_publisher(ScanPart.Feedback, robot_topic(node, '/scan/progress'), 10)
        self.roi = node.create_publisher(RoiArray, robot_topic(node, '/scan/roi'), LATCHED)
        self.result = node.create_publisher(ScanPart.Result, robot_topic(node, '/scan/result'), LATCHED)
        self.batch, self.sequence, self.sent = [], 0, 0.
        self.state = ScanPart.Feedback(stage=self.PLANNING)
        self.total = self.done = self.current = 0.  # planned, finished and current stroke length, mm
        self.speed, self.outcome = 1., {}  # outcome: ScanPart.Result fields known so far

    def header(self, message):
        message.header.stamp, message.header.frame_id = self.node.get_clock().now().to_msg(), 'base_link'
        message.run_id = self.run_id
        return message

    def plan(self, paths, speed):
        """paths: per path the stroke length of each segment (mm); speed: stroke speed (mm/s)."""
        self.total, self.done, self.speed = float(sum(map(sum, paths))), 0., speed
        self.state.paths, self.state.segments = len(paths), sum(map(len, paths))
        self.update()

    def stage(self, stage, path=None, segment=None):
        self.state.stage = stage
        self.state.path = self.state.path if path is None else path
        self.state.segment = self.state.segment if segment is None else segment
        self.update()

    def advance(self, length_mm):
        """A finished stroke of length_mm."""
        self.done, self.current = min(self.total, self.done + length_mm), 0.
        self.update()

    def within(self, length_mm):
        """Travel along the current stroke, published with the next batch."""
        self.current = length_mm

    def record(self, **fields):
        """Result fields (measured_cloud, roi_file, reports, ...) as soon as they are known."""
        self.outcome.update(fields)

    def reports(self, *paths):
        self.outcome.setdefault('reports', []).extend(map(str, paths))

    def update(self):
        self.flush()
        done = min(self.total, self.done + self.current)
        self.state.progress = done / self.total if self.total else 0.
        self.state.remaining_s = (self.total - done) / self.speed
        self.progress.publish(self.state)

    def sample(self, sampled_at, phase, axis, tcp_mm, rotation, wrench, force, contact, point_mm, points,
               sequence=None):
        """One recorded sample: phase a ContactSample constant or a PHASES name, rotation the TCP's (3x3), wrench
        [N, N m], point_mm the surface point under the ball, points the measured points kept so far."""
        self.sequence = self.sequence + 1 if sequence is None else int(sequence)
        x, y, z, w = Rotation.from_matrix(rotation).as_quat()
        self.batch.append(ContactSample(
            sampled_at=float(sampled_at), sequence=self.sequence, phase=PHASES.get(phase, phase),
            path=self.state.path, segment=self.state.segment, axis=vector(Vector3, axis),
            tcp=Pose(position=vector(Point, np.asarray(tcp_mm) / 1000), orientation=Quaternion(x=x, y=y, z=z, w=w)),
            wrench=Wrench(force=vector(Vector3, wrench[:3]), torque=vector(Vector3, wrench[3:6])),
            force_n=float(force), contact=bool(contact), point=vector(Point, np.asarray(point_mm) / 1000)))
        self.state.force_n, self.state.points = float(force), points
        if time.monotonic() - self.sent >= self.BATCH_S:
            self.update()

    def flush(self):
        if self.batch:
            self.samples.publish(self.header(ContactSampleArray(samples=self.batch)))
            self.batch = []
        self.sent = time.monotonic()

    def regions(self, regions, force_limit):
        """ROI dicts of roi.ScanRoi or staged_roi.SurveyRoi (contact loss counts as force evidence)."""
        message = self.header(RoiArray(part=self.part, force_limit_n=float(force_limit)))
        for r in sorted(regions, key=lambda r: not r['confirmed']):
            low, high = np.reshape(r['bounds_mm'], (2, 3)) / 1000
            evidence = set(r['evidence'])
            message.regions.append(RoiRegion(
                id=r['id'], type=r['type'], confirmed=bool(r['confirmed']), geometry='geometry' in evidence,
                force=bool(evidence & {'force', 'contact_loss'}), low=vector(Point, low), high=vector(Point, high),
                centre=vector(Point, (low + high) / 2), axis=vector(Vector3, r['axis']),
                area_m2=(r.get('area_mm2') or 0.) / 1e6, length_m=(r.get('length_mm') or 0.) / 1000,
                width_m=(r.get('width_mm') or 0.) / 1000, max_deviation_m=(r['max_deviation_mm'] or 0.) / 1000,
                force_residual_n=float(r['force_residual_n'] or 0.), force_samples=int(r.get('force_samples', 0))))
        self.roi.publish(message)
        confirmed = sum(region.confirmed for region in message.regions)
        self.record(confirmed_rois=confirmed, candidate_rois=len(message.regions) - confirmed)
        return message

    def finish(self, success, message):
        """The run's result, with the fields recorded on the way."""
        self.stage(self.REPORTING)
        result = ScanPart.Result(success=success, message=message, run_id=self.run_id,
                                 paths=self.state.paths, segments=self.state.segments, points=self.state.points)
        for key, value in self.outcome.items():
            setattr(result, key, value)
        self.result.publish(result)
        return result
