"""Typed ROS interface for the native position-only ProbeTCP stream."""
import json
import time

from rokey_interfaces.msg import CartesianSetpoint, CartesianState
from rokey_interfaces.srv import CartesianSession
from std_msgs.msg import String


class CartesianBridge:
    def __init__(self, bridge):
        self.bridge = bridge
        self.publisher = bridge.create_publisher(CartesianState, 'cartesian_stream/state', 1)
        self.diagnostics = bridge.create_publisher(String, 'cartesian_stream/diagnostics', 1)
        self.last_diagnostics = 0.
        self.subscription = bridge.stream_node.create_subscription(
            CartesianSetpoint, 'cartesian_stream/setpoint', self.setpoint, 1)
        self.service = bridge.create_service(CartesianSession, 'cartesian_stream/session', self.session,
                                             callback_group=bridge.services_group)

    def session(self, request, response):
        if request.command not in (request.START, request.STOP):
            response.message = 'Unknown Cartesian session command'
            return response
        start = request.command == request.START
        result = self.bridge.connection.request('cartesian_start' if start else 'cartesian_stop',
                                                {'run_id': request.run_id,
                                                 'pd': {field: getattr(request.pd, field)
                                                        for field in request.pd.get_fields_and_field_types()}})
        response.run_id = result.get('run_id', request.run_id)
        if result['success'] and not start:
            result = self.bridge.synchronize(result['sync_q'])
        response.success = result['success']
        response.message = result.get('message', '')
        return response

    def setpoint(self, message):
        b = self.bridge
        stamp = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
        age = (b.get_clock().now().nanoseconds - stamp) / 1e9
        if message.header.frame_id != 'base_link' or not 0 <= age <= .3:
            b.get_logger().error('Cartesian setpoint requires base_link and a fresh ROS stamp')
            return
        p, v = message.position, message.feedforward
        packet = dict(run_id=message.run_id, sequence=message.sequence,
                      source_sequence=message.source_sequence, source_sampled_at=message.source_sampled_at,
                      position=[p.x, p.y, p.z], rotation=list(message.rotation),
                      feedforward=[v.linear.x, v.linear.y, v.linear.z, v.angular.x, v.angular.y, v.angular.z],
                      linear_speed=message.linear_speed, linear_acceleration=message.linear_acceleration,
                      ball_radius=message.ball_radius,
                      angular_speed=message.angular_speed, angular_acceleration=message.angular_acceleration,
                      joint_speed=message.joint_speed, joint_acceleration=message.joint_acceleration,
                      position_stiffness_scale=message.position_stiffness_scale,
                      position_feedback_limit=message.position_feedback_limit,
                      contact_force=message.contact_force)
        packet['rotation'] = [packet['rotation'][i:i + 3] for i in (0, 3, 6)]
        try:
            b.connection.send({'kind': 'surface_setpoint', 'request': packet})
        except OSError as error:
            b.get_logger().error(f'Cartesian stream disconnected: {error}')

    def publish(self, line):
        d = json.loads(line)
        message = CartesianState()
        message.header.stamp = self.bridge.get_clock().now().to_msg()
        message.header.frame_id = 'base_link'
        message.run_id, message.sequence = d['run_id'], d['sequence']
        message.sampled_at = d['wall_sampled_at']
        message.ready = d.get('motion_ready', False)
        message.active = d['active'] and d['method'] == 'position'
        message.probe_attached, message.reason = d['probe_attached'], d['reason']
        message.position.x, message.position.y, message.position.z = d['position']
        message.rotation = [value for row in d['rotation'] for value in row]
        message.joint_positions = d['joint_positions']
        message.velocity.linear.x, message.velocity.linear.y, message.velocity.linear.z = d['velocity'][:3]
        message.velocity.angular.x, message.velocity.angular.y, message.velocity.angular.z = d['velocity'][3:]
        for target, values in ((message.measured_wrench, d['fast_wrench']),
                               (message.filtered_wrench, d['wrench'])):
            target.force.x, target.force.y, target.force.z = values[:3]
            target.torque.x, target.torque.y, target.torque.z = values[3:]
        message.joint_age_s, message.sensor_age_s = d['joint_age_s'], d['sensor_age_s']
        message.tracking_error_m, message.dropped_setpoints = d['tracking_error_m'], d['dropped_setpoints']
        control = d.get('control', {})
        message.pd_enabled, message.pd_saturated = control.get('pd_enabled', False), control.get('pd_saturated', False)
        for field in ('pd_error', 'pd_p', 'pd_d', 'pd_offset'):
            setattr(message, field, control.get(field, [0.] * 6))
        error = control.get('pd_velocity_error', [0.] * 6)
        message.pd_velocity_error.linear.x, message.pd_velocity_error.linear.y, message.pd_velocity_error.linear.z = error[:3]
        message.pd_velocity_error.angular.x, message.pd_velocity_error.angular.y, message.pd_velocity_error.angular.z = error[3:]
        self.publisher.publish(message)
        if message.active and time.monotonic() - self.last_diagnostics >= .1:
            self.last_diagnostics = time.monotonic()
            self.diagnostics.publish(String(data=json.dumps(dict(sampled_at=d['sampled_at'], **control))))
