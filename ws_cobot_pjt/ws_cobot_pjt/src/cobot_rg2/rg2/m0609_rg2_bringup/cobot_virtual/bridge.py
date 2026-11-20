"""Existing force services, backed by a bounded RPC to native Gazebo."""
import argparse
import json
import math
import socket
import threading
import time

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from rclpy.experimental import EventsExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.utilities import remove_ros_args
from dsr_msgs2.srv import (TaskComplianceCtrl, SetStiffnessx, SetDesiredForce,
                          ReleaseForce, ReleaseComplianceCtrl, GetToolForce,
                          CheckForceCondition, GetCurrentPosx, MoveJoint)
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from .cartesian_bridge import CartesianBridge


class GazeboConnection:
    def __init__(self, stream, token, feedback, scene, surface):
        self.stream, self.feedback, self.scene, self.surface = stream, feedback, scene, surface
        self.lock = threading.Lock()
        self.pending = {}
        self.sequence = 0
        self.closed = threading.Event()
        self.send_raw(token + '\n')
        self.reader = threading.Thread(target=self.receive, daemon=True)
        self.reader.start()

    def send_raw(self, data):
        with self.lock:
            self.stream.sendall(data.encode())

    def send(self, payload):
        self.send_raw(json.dumps(payload) + '\n')

    def request(self, command, request):
        with self.lock:
            self.sequence += 1
            sequence = self.sequence
            entry = {'event': threading.Event(), 'result': None}
            self.pending[sequence] = entry
        try:
            self.send({'kind': 'request', 'id': sequence, 'command': command, 'request': request})
            if not entry['event'].wait(10 if command == 'attach_probe' else 2):
                return {'success': False, 'message': 'Gazebo command response timed out'}
            return entry['result'] or {'success': False, 'message': 'Gazebo disconnected'}
        except OSError as error:
            return {'success': False, 'message': str(error)}
        finally:
            with self.lock:
                self.pending.pop(sequence, None)

    def receive(self):
        try:
            with self.stream.makefile('r') as file:
                for line in file:
                    if line.startswith('{"kind": "surface"'):  # 100 Hz: forwarded as is, no parse / re-encode
                        self.surface(line.rstrip('\n'))
                        continue
                    message = json.loads(line)
                    if message['kind'] == 'feedback':
                        self.feedback(message)
                    elif message['kind'] == 'scene':
                        self.scene(message['markers'])
                    elif message['kind'] == 'response':
                        with self.lock:
                            entry = self.pending.get(message['id'])
                            if entry:
                                entry['result'] = message
                                entry['event'].set()
        except OSError:
            pass
        finally:
            self.closed.set()
            with self.lock:
                for entry in self.pending.values():
                    entry['event'].set()

    def close(self):
        try:
            self.stream.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        self.stream.close()
        self.reader.join(timeout=2)


class VirtualForceBridge(Node):
    def __init__(self, stream, token):
        super().__init__('virtual_force_bridge', namespace='dsr01')
        self.positions = {}
        self.nominal_time = 0.0
        self.state = {}
        self.state_time = 0.0
        self.condition = threading.Condition()
        self.services_group = MutuallyExclusiveCallbackGroup()
        self.client_group = ReentrantCallbackGroup()
        self.pose_group = MutuallyExclusiveCallbackGroup()
        # Read-only force queries answer from the cached state: never queued behind the blocking services (surface
        # end ~7 s, probe attach 10 s, compliance waits 2 s) of the mutually exclusive services group.
        self.query_group = ReentrantCallbackGroup()
        self.positions_lock, self.positions_ready = threading.Lock(), threading.Event()
        self.joints = self.create_publisher(JointState, 'virtual_control/joint_states', 10)
        self.sim_joints = self.create_publisher(JointState, 'sim_joint_states', 10)
        self.wrench = self.create_publisher(WrenchStamped, 'virtual_control/wrench', 10)
        self.status = self.create_publisher(String, 'virtual_control/status', 10)
        self.surface_state = self.create_publisher(String, 'surface_follow/state', 1)
        self.scene = self.create_publisher(MarkerArray, 'sim_scene',
                                           QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.connection = GazeboConnection(stream, token, self.feedback, self.publish_scene, self.publish_surface)
        # The 150 Hz joint states and 40 Hz setpoints on their own service-free node, spun by an EventsExecutor (main):
        # the MultiThreadedExecutor rebuilt the wait set of all ~30 entities in Python for every message, one core
        # (531 s of CPU in 11 min) holding the GIL from the surface state / setpoint threads (state gaps up to
        # 197 ms, watchdog / stale aborts). Services stay here, where they block on native responses.
        self.stream_node = rclpy.create_node('virtual_force_stream', namespace=self.get_namespace(),
                                             start_parameter_services=False)
        self.subscribers = [self.stream_node.create_subscription(JointState, topic, self.receive_joints,
                                                                 qos_profile_sensor_data)
                            for topic in ['nominal_joint_states', 'gripper_joint_states']]
        threading.Thread(target=self.forward_positions, daemon=True).start()
        self.sync_client = self.create_client(MoveJoint, 'dsr_controller2/motion/move_joint',
                                              callback_group=self.client_group)
        self.services_kept = []
        self.services_kept.append(self.create_service(
            Trigger, 'probe/attach', self.attach_probe, callback_group=self.services_group))
        self.surface_commands = self.stream_node.create_subscription(String, 'surface_follow/setpoint',
                                                                     self.surface_setpoint, 1)
        self.cartesian = CartesianBridge(self)
        for method in ('hybrid', 'end'):
            def surface_callback(request, response, name=method):
                result = self.connection.request('surface_' + name, {})
                if result['success'] and name == 'end':
                    result = self.synchronize(result['sync_q'])
                response.success, response.message = result['success'], json.dumps(result)
                return response
            self.services_kept.append(self.create_service(
                Trigger, 'surface_follow/' + method, surface_callback,
                callback_group=self.services_group))
        for kind, name in [(TaskComplianceCtrl, 'task_compliance_ctrl'),
                           (SetStiffnessx, 'set_stiffnessx'),
                           (SetDesiredForce, 'set_desired_force'),
                           (ReleaseForce, 'release_force'),
                           (ReleaseComplianceCtrl, 'release_compliance_ctrl')]:
            def callback(request, response, command=name):
                return self.command(command, request, response)
            self.services_kept.append(self.create_service(
                kind, 'dsr_controller2/force/' + name, callback,
                callback_group=self.services_group))
        self.services_kept.append(self.create_service(
            GetToolForce, 'dsr_controller2/aux_control/get_tool_force', self.get_force,
            callback_group=self.query_group))
        self.services_kept.append(self.create_service(
            CheckForceCondition, 'dsr_controller2/force/check_force_condition', self.check_force,
            callback_group=self.query_group))
        self.posx_client = self.create_client(GetCurrentPosx, 'dsr_controller2/vendor/aux_control/get_current_posx',
                                              callback_group=self.client_group)
        self.services_kept.append(self.create_service(
            GetCurrentPosx, 'dsr_controller2/aux_control/get_current_posx', self.get_posx,
            callback_group=self.pose_group))

    def receive_joints(self, message):
        with self.positions_lock:
            self.positions.update(zip(message.name, message.position))
            if 'joint_1' in message.name:
                self.nominal_time = time.monotonic()
        self.positions_ready.set()

    def attach_probe(self, request, response):
        result = self.connection.request('attach_probe', {})
        response.success = result['success']
        response.message = result.get('message', '')
        return response

    def forward_positions(self):
        """DSR joint targets to native Gazebo as they arrive, latest only, at most 60 Hz; every 0.1 s without new
        targets too, so native still sees them go stale ('fresh'). Was a 60 Hz timer in the joint subscriptions'
        callback group."""
        sent = 0.
        while not self.connection.closed.is_set():
            self.positions_ready.wait(.1)
            time.sleep(max(0., sent + 1 / 60 - time.monotonic()))
            self.positions_ready.clear()
            with self.positions_lock:
                if not self.positions:
                    continue
                payload = {'kind': 'positions', 'positions': dict(self.positions),
                           'fresh': time.monotonic() - self.nominal_time < 0.5}
            try:
                self.connection.send(payload)
            except OSError:
                return
            sent = time.monotonic()

    def surface_setpoint(self, message):
        try:
            packet = json.loads(message.data)
            age = (self.get_clock().now().nanoseconds - int(packet['stamp_ns'])) / 1e9
            if not 0 <= age <= .15:
                raise ValueError('Expired surface command at ROS bridge')
            self.connection.send({'kind': 'surface_setpoint', 'request': packet})
        except (ValueError, KeyError, TypeError, OSError) as error:
            self.get_logger().error(f'Surface setpoint rejected: {error}')

    def feedback(self, message):
        if not rclpy.ok():
            return
        with self.condition:
            self.state, self.state_time = message, time.monotonic()
            self.condition.notify_all()
        stamp = self.get_clock().now().to_msg()
        positions = message['positions']
        joints = JointState()
        joints.header.stamp = stamp
        joints.name = [f'joint_{index}' for index in range(1, 7)
                       if f'joint_{index}' in positions]
        joints.position = [positions[name] for name in joints.name]
        self.joints.publish(joints)
        measured = JointState()
        measured.header.stamp = stamp
        measured.header.frame_id = 'base_link'
        measured.name = list(positions)
        measured.position = [positions[name] for name in measured.name]
        self.sim_joints.publish(measured)
        wrench = WrenchStamped()
        wrench.header.stamp, wrench.header.frame_id = stamp, 'base_link'
        values = message['wrench']
        wrench.wrench.force.x, wrench.wrench.force.y, wrench.wrench.force.z = values[:3]
        wrench.wrench.torque.x, wrench.wrench.torque.y, wrench.wrench.torque.z = values[3:]
        self.wrench.publish(wrench)
        self.status.publish(String(data=json.dumps({key: message[key]
                            for key in ['ready', 'mode', 'error', 'settled',
                                        'probe_attached']} | {
                                            'gripper_angle_rad': positions.get('rg2_finger_joint')})))

    def publish_surface(self, line):
        """Surface stream state, sent by the native control loop each tick (not with the display feedback)."""
        if rclpy.ok():
            self.surface_state.publish(String(data=line))
            if hasattr(self, 'cartesian'):
                self.cartesian.publish(line)

    def publish_scene(self, markers):
        """Selected map scene (BASE frame) from the native session; empty without a map."""
        stamp, array = self.get_clock().now().to_msg(), MarkerArray()
        for item in markers:
            marker = Marker(ns=item['ns'], id=item['id'], type=item['type'], action=Marker.ADD,
                            mesh_resource=item['mesh'])
            marker.header.stamp, marker.header.frame_id = stamp, 'base_link'
            position, orientation = marker.pose.position, marker.pose.orientation
            (position.x, position.y, position.z,
             orientation.x, orientation.y, orientation.z, orientation.w) = item['pose']
            marker.scale.x, marker.scale.y, marker.scale.z = item['scale']
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = item['color']
            array.markers.append(marker)
        self.scene.publish(array)

    def command(self, name, request, response):
        if name == 'task_compliance_ctrl':
            with self.condition:
                settled = self.condition.wait_for(
                    lambda: self.state.get('ready') and self.state.get('settled'), timeout=2)
            if not settled:
                self.get_logger().error('Gazebo must settle before enabling compliance control')
                response.success = False
                return response
        fields = {}
        for field in request.get_fields_and_field_types():
            value = getattr(request, field)
            fields[field] = value.tolist() if hasattr(value, 'tolist') else value
        result = self.connection.request(name, fields)
        if result['success'] and name == 'release_compliance_ctrl':
            result = self.synchronize(result['sync_q'])
        response.success = result['success']
        if not response.success:
            self.get_logger().error(result.get('message', 'Virtual force command failed'))
        return response

    def synchronize(self, joints):
        if not self.sync_client.wait_for_service(timeout_sec=1):
            return {'success': False, 'message': 'Emulator joint synchronization service unavailable'}
        future = self.sync_client.call_async(MoveJoint.Request(
            pos=[math.degrees(value) for value in joints], vel=30., acc=60.,
            time=0., radius=0., mode=0, blend_type=0, sync_type=0))
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(5) or not future.result().success:
            return {'success': False, 'message': 'Emulator joint synchronization failed; Gazebo holds position'}
        result = self.connection.request('resume', {})
        if not result['success']:
            return result
        with self.condition:
            followed = self.condition.wait_for(lambda: self.state.get('mode') == 'following', timeout=2)
        return {'success': followed, 'message': 'Waiting for emulator pose synchronization'}

    def force_values(self, reference):
        if time.monotonic() - self.state_time > 0.5 or not self.state.get('ready'):
            raise ValueError('Gazebo force feedback is unavailable or stale')
        wrench = self.state['wrench']
        matrix = self.state['tool_rotation']
        def transpose_rotate(vector):
            return [sum(matrix[row][column] * vector[row] for row in range(3))
                    for column in range(3)]
        if reference not in (0, 1, 2):
            raise ValueError('Unsupported force reference')
        force = transpose_rotate(wrench[:3]) if reference == 1 else wrench[:3]
        return force + transpose_rotate(wrench[3:])

    def get_force(self, request, response):
        try:
            response.tool_force = self.force_values(request.ref)
            response.success = True
        except ValueError as error:
            self.get_logger().error(str(error))
            response.success = False
        return response

    def get_posx(self, request, response):
        # Keep the vendor solution-space field; BASE pose comes directly from the physical TCP model.
        future = self.posx_client.call_async(GetCurrentPosx.Request(ref=request.ref))
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        result = future.result() if done.wait(2) else None
        response.success = bool(result and result.success)
        if response.success:
            data = list(result.task_pos_info[0].data)
            if request.ref == 0:
                data[:6] = self.state['tcp_pose_base_mm_deg']
            else:
                data[:3] = [value + delta for value, delta in zip(data[:3], self.state.get('tcp_offset_mm', [0.0] * 3))]
            result.task_pos_info[0].data = data
            response.task_pos_info = result.task_pos_info
        return response

    def check_force(self, request, response):
        try:
            values = self.force_values(request.ref)
            axes = {0: 0, 1: 1, 2: 2, 10: 3, 11: 4, 12: 5}
            value = abs(values[axes[request.axis]])
            response.success = ((request.min < 0 or value >= request.min) and
                                (request.max < 0 or value <= request.max))
        except (ValueError, KeyError):
            response.success = False
        return response


def main(bridge=VirtualForceBridge):
    parser = argparse.ArgumentParser()
    parser.add_argument('host')
    parser.add_argument('port', type=int)
    parser.add_argument('token')
    args = parser.parse_args(remove_ros_args()[1:])
    rclpy.init()
    stream = socket.create_connection((args.host, args.port), timeout=5)
    stream.settimeout(2)
    stream.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    node = bridge(stream, args.token)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    # spin() keeps the worker pool fed; spin_once(0.1) in a loop dispatched one callback per call.
    threading.Thread(target=executor.spin, daemon=True).start()
    stream_executor = EventsExecutor()
    stream_executor.add_node(node.stream_node)
    threading.Thread(target=stream_executor.spin, daemon=True).start()
    try:
        while rclpy.ok() and not node.connection.closed.wait(.5):
            pass
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.connection.close()
        executor.shutdown()
        stream_executor.shutdown()
        node.stream_node.destroy_node()
        node.destroy_node()
        rclpy.try_shutdown()
