"""Select the Gazebo probe or gripper profiles on the current robot's controller."""
from dsr_msgs2 import srv


class VirtualProbeProfiles:
    def __init__(self, robot, plan):
        self.robot, self.plan = robot, plan

    def select(self, *, attached, refresh=False):
        robot, plan = self.robot, self.plan
        if not plan.get('simulation_only') or robot.get_robot_system() != 1:
            raise RuntimeError('The Gazebo TCP/Tool can only be set on the virtual robot.')
        initial = plan['initial_profiles']
        tcp, tool = (plan['tcp_name'], plan['tool_name']) if attached else (initial['tcp'], initial['tool'])
        if not refresh and (robot.get_tcp(), robot.get_tool()) == (tcp, tool):
            return
        if robot.check_motion() != 0:
            raise RuntimeError('The robot must be stopped to change the Gazebo TCP/Tool.')
        mode = robot.get_robot_mode()
        robot.request('system/set_robot_mode', srv.SetRobotMode, robot_mode=0)
        try:
            if attached:
                # Apply the actual mounted transform from this robot's session plan.
                robot.set_tcp(initial['tcp'])
                robot.set_tool(initial['tool'])
                robot.request('tcp/config_delete_tcp', srv.ConfigDeleteTcp, name=tcp)
                robot.request('tcp/config_create_tcp', srv.ConfigCreateTcp, name=tcp, pos=plan['tcp_mm_deg'])
                robot.request('tool/config_delete_tool', srv.ConfigDeleteTool, name=tool)
                robot.request('tool/config_create_tool', srv.ConfigCreateTool, name=tool, **plan['tool_profile'])
            robot.set_tool(tool)
            robot.set_tcp(tcp)
        finally:
            robot.request('system/set_robot_mode', srv.SetRobotMode, robot_mode=mode)
        robot.verify_profiles(tool, tcp)
