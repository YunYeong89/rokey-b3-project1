"""ROS 2 bridge to the turn-lift firmware's TCP server (firmware/car_turn_lift.py --serve, on the Raspberry Pi or the
native Gazebo lift of run_gazebo.sh):
  /lift/command  custom_interfaces/srv/LiftCommand  one firmware command line, answered when the motion has finished
  /lift/status   custom_interfaces/msg/LiftStatus   after every command and every `status_period` s while idle
Parameters: host (Raspberry Pi IP, or host.docker.internal for the Gazebo firmware on the Mac), port, timeout s,
status_period s.  ros2 run repair_car lift_bridge --ros-args -p host:=192.168.0.10"""
import math
import threading

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from custom_interfaces.msg import LiftStatus
from custom_interfaces.srv import LiftCommand

from .firmware import FirmwareClient


class LiftBridge(Node):
    def __init__(self):
        super().__init__('lift')
        host = self.declare_parameter('host', 'host.docker.internal').value
        port = self.declare_parameter('port', 5005).value
        timeout = self.declare_parameter('timeout', 60.).value
        period = self.declare_parameter('status_period', 1.).value
        self.client, self.lock = FirmwareClient(host, port, timeout), threading.Lock()
        group = ReentrantCallbackGroup()   # the status poll skips while a command holds the firmware
        self.publisher = self.create_publisher(LiftStatus, '/lift/status', 10)
        self.create_service(LiftCommand, '/lift/command', self.on_command, callback_group=group)
        self.create_timer(period, self.poll, callback_group=group)
        self.get_logger().info(f'firmware {host}:{port}')

    def request(self, line):
        try:
            return self.client.send(line)
        except OSError as error:
            return {'ok': False, 'error': f'firmware {self.client.address[0]}:{self.client.address[1]}: {error}'}

    def on_command(self, request, response):
        with self.lock:
            reply = self.request(request.command)
        response.ok, response.error = reply['ok'], reply.get('error', '')
        if reply['ok']:
            response.status = self.status(reply)
            self.publisher.publish(response.status)
        self.get_logger().info(f'{request.command}: {reply}')
        return response

    def poll(self):
        if not self.lock.acquire(blocking=False):
            return
        try:
            reply = self.request('status')
        finally:
            self.lock.release()
        if reply['ok']:
            self.publisher.publish(self.status(reply))
        else:
            self.get_logger().warn(reply['error'], throttle_duration_sec=10.)

    @staticmethod
    def status(reply):
        return LiftStatus(heading=float(reply['heading']), station=reply['station'], lift_deg=float(reply['lift_deg']),
                          lift_mm=float(reply.get('lift_mm', math.nan)),
                          stand_top_mm=float(reply.get('stand_top_mm', math.nan)),
                          vehicle_z_mm=float(reply.get('vehicle_z_mm', math.nan)))


def main(args=None):
    rclpy.init(args=args)
    node = LiftBridge()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
