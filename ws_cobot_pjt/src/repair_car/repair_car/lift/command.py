"""ros2 run repair_car lift_command COMMAND...  one call of lift/command (custom_interfaces/srv/LiftCommand), e.g. `lift up`,
`lift 125`, `heading 90`, `status`; the reply printed as one line. /lift/command is the real lift_bridge; a virtual
demo_final stand answers in its robot namespace: ros2 run repair_car lift_command status --ros-args -r __ns:=/robot_2"""
import sys

import rclpy
from rclpy.utilities import remove_ros_args
from custom_interfaces.srv import LiftCommand


def main(args=None):
    rclpy.init(args=args)
    line = ' '.join(remove_ros_args(sys.argv)[1:])
    node = rclpy.create_node('lift_command')
    client = node.create_client(LiftCommand, 'lift/command')
    try:
        if not client.wait_for_service(timeout_sec=10.):
            sys.exit(f'{client.srv_name} service missing (ros2 run repair_car lift_bridge, or a virtual demo_final session)')
        future = client.call_async(LiftCommand.Request(command=line))
        rclpy.spin_until_future_complete(node, future)
        reply, status = future.result(), future.result().status
        print(f'{line}: ' + (f'heading {status.heading:.1f}, station {status.station}, lift {status.lift_deg:.1f} deg '
                             f'{status.lift_mm:.1f} mm, stand top {status.stand_top_mm:.1f} mm, vehicle z {status.vehicle_z_mm:.2f} mm'
                             if reply.ok else f'failed {reply.error}'))
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
