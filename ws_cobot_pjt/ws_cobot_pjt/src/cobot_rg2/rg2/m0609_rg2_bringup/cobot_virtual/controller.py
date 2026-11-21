"""Container side of the accelerated MuJoCo session (run_virtual.sh --acc): no Doosan emulator and no
dsr_controller2. This bridge serves the dsr_controller2 services the robot programs call (system, motion, tool / TCP,
pose), the RG2 command (/onrobot/sendCommand) and the controller alarms (/dsr01/error) from the native emulated
controller (dsr_mujoco/controller.py), next to the force services of the virtual force bridge.
  python3 -m cobot_virtual.controller HOST PORT TOKEN   (started by the native session)"""
import time
from functools import partial

from dsr_msgs2 import srv
from dsr_msgs2.msg import RobotError
from onrobot_rg_msgs.srv import SetCommand
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Float64MultiArray

from .bridge import VirtualForceBridge, main as run

SERVICES = {
    'system/get_robot_system': srv.GetRobotSystem,
    'system/get_robot_mode': srv.GetRobotMode,
    'system/set_robot_mode': srv.SetRobotMode,
    'motion/move_joint': srv.MoveJoint,
    'motion/move_line': srv.MoveLine,
    'motion/check_motion': srv.CheckMotion,
    'motion/move_stop': srv.MoveStop,
    'motion/move_wait': srv.MoveWait,
    'motion/fkin': srv.Fkin,
    'motion/ikin': srv.Ikin,
    'motion/set_ref_coord': srv.SetRefCoord,
    'tool/get_current_tool': srv.GetCurrentTool,
    'tool/set_current_tool': srv.SetCurrentTool,
    'tool/config_create_tool': srv.ConfigCreateTool,
    'tcp/get_current_tcp': srv.GetCurrentTcp,
    'tcp/set_current_tcp': srv.SetCurrentTcp,
    'tcp/config_create_tcp': srv.ConfigCreateTcp,
    'aux_control/get_current_posj': srv.GetCurrentPosj,
}


class ControllerBridge(VirtualForceBridge):
    def __init__(self, stream, token):
        super().__init__(stream, token)
        for subscription in self.subscribers:  # no emulator joint stream: the native controller makes the nominal
            self.stream_node.destroy_subscription(subscription)
        self.alarm_id = 0
        self.errors = self.create_publisher(RobotError, 'error', 10)
        group = ReentrantCallbackGroup()  # sync moves / move_wait block their thread only
        self.services_kept += [self.create_service(kind, 'dsr_controller2/' + path, partial(self.forward, path),
                                                   callback_group=group) for path, kind in SERVICES.items()]
        self.services_kept.append(self.create_service(SetCommand, '/onrobot/sendCommand', self.gripper,
                                                      callback_group=group))

    @staticmethod
    def fields(request):
        return {name: (value.tolist() if hasattr(value, 'tolist') else value)
                for name in request.get_fields_and_field_types() for value in [getattr(request, name)]}

    def forward(self, path, request, response):
        if path == 'motion/move_wait':
            response.success = self.wait_motion()
            return response
        result = self.connection.request(path, self.fields(request))
        for name, value in result.get('values', {}).items():
            setattr(response, name, value)
        response.success = bool(result['success'])
        if not response.success:
            self.get_logger().error(f'{path}: {result.get("message", "refused")}')
        elif getattr(request, 'sync_type', 1) == 0:
            response.success = self.wait_motion()
        return response

    def wait_motion(self, timeout=600.):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            result = self.connection.request('motion/check_motion', {})
            if not result['success'] or result['values']['status'] == 0:
                return bool(result['success'])
            time.sleep(.02)
        return False

    def get_posx(self, request, response):
        result = self.connection.request('aux_control/get_current_posx', {'ref': request.ref})
        response.success = bool(result['success'])
        if response.success:
            response.task_pos_info = [Float64MultiArray(data=result['values']['posx'])]
        return response

    def synchronize(self, joints):
        """After compliance / the surface stream the native controller resumes from the held joints itself."""
        result = self.connection.request('resume', {})
        if not result['success']:
            return result
        with self.condition:
            followed = self.condition.wait_for(lambda: self.state.get('mode') == 'following', timeout=2)
        return {'success': followed, 'message': 'Waiting for the controller to resume'}

    def gripper(self, request, response):
        result = self.connection.request('gripper', {'command': request.command})
        response.success, response.message = bool(result['success']), result.get('message', '')
        if response.success:
            target = result['values']['target']
            with self.condition:
                response.success = self.condition.wait_for(
                    lambda: abs(self.state.get('controller', {}).get('gripper', target + 1) - target) < .01,
                    timeout=30)
        return response

    def feedback(self, message):
        super().feedback(message)
        alarm = message.get('controller', {}).get('alarm', {})
        if alarm.get('id', 0) > self.alarm_id:
            self.alarm_id = alarm['id']
            self.errors.publish(RobotError(level=alarm['level'], group=alarm['group'], code=alarm['code'],
                                           msg1=alarm['text']))


def main():
    run(ControllerBridge)


if __name__ == '__main__':
    main()
