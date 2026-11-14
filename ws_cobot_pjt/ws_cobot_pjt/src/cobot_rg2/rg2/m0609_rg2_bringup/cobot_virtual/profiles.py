"""Register configured TCP/tool profiles on a virtual controller only."""
import math
import json
from pathlib import Path

import rclpy
import yaml
from dsr_msgs2.srv import (
    ConfigCreateTcp, ConfigCreateTool, GetCurrentTcp, GetCurrentTool,
    GetRobotMode, GetRobotSystem, SetCurrentTcp, SetCurrentTool, SetRobotMode,
)
from rclpy.node import Node


class VirtualProfileLoader(Node):
    def __init__(self):
        super().__init__('virtual_profile_loader')
        self.declare_parameter('profile_file', '')
        path = Path(self.get_parameter('profile_file').value)
        contents = path.read_text(encoding='utf-8')
        self.profiles = json.loads(contents) if path.suffix == '.json' else yaml.safe_load(contents)
        self.get_logger().info(f'Loading virtual Tool/TCP profiles: {path}')

    def call(self, service, path, **values):
        client = self.create_client(service, f'dsr_controller2/{path}')
        try:
            if not client.wait_for_service(timeout_sec=10.0):
                raise RuntimeError(f'Service unavailable: {path}')
            future = client.call_async(service.Request(**values))
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            if not future.done():
                raise RuntimeError(f'Service timed out: {path}')
            response = future.result()
            if not response.success:
                raise RuntimeError(f'Service rejected {path}: {values}')
            return response
        finally:
            self.destroy_client(client)

    @staticmethod
    def vector(values, length):
        if len(values) != length:
            raise ValueError(f'Expected {length} values: {values}')
        result = [float(value) for value in values]
        if not all(math.isfinite(value) for value in result):
            raise ValueError(f'Non-finite profile values: {values}')
        return result

    def load(self):
        if self.call(GetRobotSystem, 'system/get_robot_system').robot_system != 1:
            raise RuntimeError('Profile registration is restricted to virtual robots')
        tcps, tools = {}, {}
        for name, profile in self.profiles['tcps'].items():
            tcps[name] = self.vector(profile['pos'], 6)
        for name, profile in self.profiles['tools'].items():
            weight = float(profile['weight'])
            if not math.isfinite(weight) or weight < 0:
                raise ValueError(f'Invalid tool weight: {name}')
            tools[name] = dict(weight=weight, cog=self.vector(profile['cog'], 3),
                               inertia=self.vector(profile['inertia'], 6)
                               if profile['inertia'] is not None else [0.0] * 6)
        active = self.profiles['active']
        if active['tcp'] not in tcps or active['tool'] not in tools:
            raise ValueError(f'Unknown active profiles: {active}')
        mode = self.call(GetRobotMode, 'system/get_robot_mode').robot_mode
        self.call(SetRobotMode, 'system/set_robot_mode', robot_mode=0)
        try:
            for name, pos in tcps.items():
                self.call(ConfigCreateTcp, 'tcp/config_create_tcp', name=name, pos=pos)
                self.get_logger().info(f'Registered TCP {name}: {pos}')
            for name, values in tools.items():
                self.call(ConfigCreateTool, 'tool/config_create_tool', name=name, **values)
                self.get_logger().info(f'Registered Tool {name}: {values}')
            self.call(SetCurrentTcp, 'tcp/set_current_tcp', name=active['tcp'])
            self.call(SetCurrentTool, 'tool/set_current_tool', name=active['tool'])
        finally:
            self.call(SetRobotMode, 'system/set_robot_mode', robot_mode=mode)
        tcp = self.call(GetCurrentTcp, 'tcp/get_current_tcp').info
        tool = self.call(GetCurrentTool, 'tool/get_current_tool').info
        if (tcp, tool) != (active['tcp'], active['tool']):
            raise RuntimeError(f'Active profile mismatch: TCP={tcp}, Tool={tool}')
        self.get_logger().info(f'Virtual profiles ready: TCP={tcp}, Tool={tool}')


def main():
    rclpy.init()
    node = VirtualProfileLoader()
    try:
        node.load()
    except Exception as error:
        node.get_logger().error(str(error))
        raise SystemExit(1) from error
    finally:
        node.destroy_node()
        rclpy.shutdown()
