"""Typed position-stream client. Force samples are never part of readiness or motion decisions."""
import time

import numpy as np
from rokey_interfaces.msg import CartesianSetpoint, CartesianState, PosePdConfig
from rokey_interfaces.srv import CartesianSession

# The native controller rejects a setpoint whose ball-centre feedforward exceeds the speed sent with it. The path timing
# cruises at 99.9 % of the detection speed on its grid, but between grid points at tight turns the feedforward reached
# +0.13 % (bonnet front edge), and the stream stopped there. The limit sent is a safety cap well above the cruise:
# SPEED_LIMIT_MM_S, or the detection speed plus SPEED_LIMIT_HEADROOM when that is faster (--speed).
SPEED_LIMIT_MM_S = 10.0
SPEED_LIMIT_HEADROOM = 1.1


class CartesianTransport:
    def __init__(self, program, settings, detection_speed_mm_s, *, force_axis=None):
        self.program, self.robot, self.node, self.settings = program, program.robot, program.node, settings
        self.force_axis = force_axis
        self.linear_speed = max(SPEED_LIMIT_MM_S, detection_speed_mm_s * SPEED_LIMIT_HEADROOM) / 1000
        self.message, self.received = None, 0.
        self.run_id, self.sequence = '', 0
        self.publisher = self.node.create_publisher(CartesianSetpoint, 'cartesian_stream/setpoint', 1)
        self.subscription = self.node.create_subscription(CartesianState, 'cartesian_stream/state', self.receive, 1)
        self.client = self.node.create_client(CartesianSession, 'cartesian_stream/session')

    def receive(self, message):
        self.message, self.received = message, time.monotonic()

    def spin(self, timeout=.005):
        self.robot.executor.spin_once(timeout_sec=timeout)

    def state(self):
        m = self.message
        if m is None or time.monotonic() - self.received > .3 or not m.ready or m.joint_age_s > .1:
            raise TimeoutError('Cartesian joint/pose feedback is unavailable or stale')
        return m

    def wait_ready(self, timeout=5.):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.spin(.02)
            if self.message is not None and time.monotonic() - self.received < .1 and self.message.ready:
                return self.state()
        raise TimeoutError('Cartesian stream unavailable: check the selected backend, feedback and probe status')

    def begin(self):
        self.robot.check_cancelled()
        response = self.session(CartesianSession.Request.START, '')
        self.run_id = response.run_id
        deadline = time.monotonic() + .2
        while time.monotonic() < deadline:
            self.spin(.005)
            if self.message is not None and self.message.run_id == self.run_id and self.message.active:
                if self.message.pd_enabled != self.settings['pd']['enabled']:
                    raise RuntimeError('Cartesian PD activation does not match the request')
                return
        raise TimeoutError('Cartesian stream did not report the new session')

    def wait_still(self, timeout=3.):
        deadline, stable = time.monotonic() + timeout, None
        while time.monotonic() < deadline:
            self.spin(.01)
            state = self.state()
            v = state.velocity
            still = (np.linalg.norm([v.linear.x, v.linear.y, v.linear.z]) < .0008 and
                     np.linalg.norm([v.angular.x, v.angular.y, v.angular.z]) < .01)
            stable = (stable or time.monotonic()) if still else None
            if stable is not None and time.monotonic() - stable > .15:
                return state
        raise TimeoutError('TCP did not become stationary before detection')

    def send(self, position, rotation, feed, force=0.):
        """Force target on the session's press axis, in N. Zero requests position-only motion."""
        state = self.state()
        if state.run_id != self.run_id or not state.active:
            raise RuntimeError(state.reason or 'Cartesian stream ownership changed')
        self.sequence += 1
        message = CartesianSetpoint()
        message.header.stamp, message.header.frame_id = self.node.get_clock().now().to_msg(), 'base_link'
        message.run_id, message.sequence = self.run_id, self.sequence
        message.source_sequence, message.source_sampled_at = state.sequence, state.sampled_at
        message.position.x, message.position.y, message.position.z = map(float, np.asarray(position) / 1000)
        message.rotation = np.asarray(rotation).ravel().tolist()
        message.feedforward.linear.x, message.feedforward.linear.y, message.feedforward.linear.z = map(float, feed[:3] / 1000)
        message.feedforward.angular.x, message.feedforward.angular.y, message.feedforward.angular.z = map(float, feed[3:])
        c = self.settings
        message.linear_speed = self.linear_speed
        message.linear_acceleration = c['linear_acceleration_mm_s2'] / 1000
        message.ball_radius = c.get('ball_radius_mm', 0.) / 1000
        message.angular_speed = float(np.radians(c['angular_speed_deg_s']))
        # The command limiter carries the PD correction on top of the planned motion: more headroom than the plan.
        message.angular_acceleration = float(np.radians(c['pd']['command_angular_acceleration_deg_s2']))
        message.joint_speed = float(np.radians(c['joint_speed_deg_s']))
        message.joint_acceleration = float(np.radians(c['pd']['command_joint_acceleration_deg_s2']))
        message.position_stiffness_scale = c['position_stiffness_scale']
        message.position_feedback_limit = c['position_feedback_limit_mm'] / 1000
        message.contact_force = float(force)
        self.publisher.publish(message)

    def end(self):
        if self.run_id:
            # Cleanup must still reach the server after the program receives cancellation.
            self.session(CartesianSession.Request.STOP, self.run_id)
            self.run_id = ''

    def session(self, command, run_id):
        if not self.client.wait_for_service(timeout_sec=3.):
            raise TimeoutError('Cartesian session service unavailable')
        request = CartesianSession.Request(command=command, run_id=run_id)
        if command == CartesianSession.Request.START:
            c = self.settings['pd']
            if self.force_axis is not None:
                request.force_axis.x, request.force_axis.y, request.force_axis.z = map(float, self.force_axis)
            request.pd = PosePdConfig(
                enabled=c['enabled'], position_kp=c['position_kp'], position_kd_s=c['position_kd_s'],
                orientation_kp=c['orientation_kp'], orientation_kd_s=c['orientation_kd_s'],
                derivative_filter_s=c['derivative_filter_s'],
                position_feedback_speed=c['position_feedback_speed_mm_s'] / 1000,
                angular_feedback_limit=float(np.radians(c['angular_feedback_limit_deg'])),
                angular_feedback_speed=float(np.radians(c['angular_feedback_speed_deg_s'])))
        future = self.client.call_async(request)
        deadline = time.monotonic() + 10.
        while not future.done() and time.monotonic() < deadline:
            self.spin(.02)
        if not future.done():
            future.cancel()
            raise TimeoutError('Cartesian session response timed out')
        response = future.result()
        if not response.success:
            raise RuntimeError(response.message)
        return response

    def close(self):
        self.node.destroy_subscription(self.subscription)
        self.node.destroy_publisher(self.publisher)
        self.node.destroy_client(self.client)
