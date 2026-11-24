#!/usr/bin/env python3
"""Bonnet disassembly and assembly (pendant DRL ported to DSR_ROBOT2 Python)

Source:
  drl/m0609_bonnet.drl          → disassemble()
  drl/m0609_bonnet_assembly.drl → assemble()

Run:
  ros2 run repair_car bonnet_replace <disassembly|assembly|all> [--virtual] [--until N]

Use from other code (e.g. replace_parts):
  the caller sets DR_init.__dsr__id / __dsr__model / __dsr__node first, then
  from repair_car.part_replace.bonnet_replace import disassemble, assemble
  disassemble(); assemble()
  * on failure or stop: cleanup (stop motion → release force → release compliance → step log), then re-raise.
  * set_velj/accj/velx/accx are DSR_ROBOT2 globals, so the default speeds of later motions change too.

Porting rules:
  - coordinates, order, ref (0 = DR_BASE), mod, stiffness, force and release order as in the DRL.
  - while gLoop < 1 → run once.
  - differences from the installed DSR_ROBOT2:
      * movel has no app_type argument → the DRL's app_type=DR_MV_APP_NONE (no approach motion) is dropped.
      * set_velx takes only 2 arguments and there is no DR_OFF constant → the DRL's 3rd argument DR_OFF is dropped.
  - every DSR command's return value (-1 = failure) is checked; on failure it stops instead of going to the next line.
  - 1 s wait after gripper open/close commands. The real robot only checks the probe grasp before disassembly.
  - Places commented "differs from the source: safety" deviate from the source DRL.
  - no automatic return home on an error/Ctrl+C (return with the pendant).
"""
import argparse
import sys

import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args
import DR_init

from repair_car.config import load
from repair_car.robot_control.rg2_state import RG2State

# ---------------------------------------------------------------- settings
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

TOOL_NAME = "Tool Weight v2"
TCP_NAME = "GripperDA_v1_probe"

SPEED_SCALE = 0.2                                        # ratio to the DRL default speed
VELJ = 60.0 * SPEED_SCALE                                # 12.0
ACCJ = 100.0 * SPEED_SCALE                               # 20.0
VELX = (250.0 * SPEED_SCALE, 80.625 * SPEED_SCALE)       # (50.0, 16.125)
ACCX = (1000.0 * SPEED_SCALE, 322.5 * SPEED_SCALE)       # (200.0, 64.5)
# movel with time= has its time divided by SPEED_SCALE, slowing it by the same ratio
T_FORCE_MOVE = 9.33 / SPEED_SCALE                        # 46.65 [s] (disassembly DRL lines 36, 38)

GRIP_WAIT = 1.0                                          # wait after a gripper action [s]

MOVE_STOP_SERVICE = "dsr_controller2/motion/move_stop"
MOVE_STOP_MODE = 1                                       # DR_QSTOP (Stop Category 2)


class StepFailed(RuntimeError):
    """A DSR command failed (-1) or a start condition is not met."""


class StopAfterStep(Exception):
    """--until N: stop on purpose after step N (not an error)."""


def _ok(ret, what):
    if ret != 0:
        raise StepFailed(f"{what} failed (ret={ret})")


# ---------------------------------------------------------------- gripper
def grip_close():
    """DO1 ON, DO2 OFF (the caller waits)."""
    from DSR_ROBOT2 import set_digital_output, ON, OFF
    _ok(set_digital_output(2, OFF), "set_digital_output(2, OFF)")
    _ok(set_digital_output(1, ON), "set_digital_output(1, ON)")


def grip_open():
    """DO1 OFF, DO2 ON (the caller waits)."""
    from DSR_ROBOT2 import set_digital_output, ON, OFF
    _ok(set_digital_output(1, OFF), "set_digital_output(1, OFF)")
    _ok(set_digital_output(2, ON), "set_digital_output(2, ON)")


# ---------------------------------------------------------------- run state / cleanup
class _Run:
    """Step log, force/compliance state, cleanup on failure."""

    def __init__(self, label, total, until=None):
        self.until = until                    # None: to the end, N: stop after step N
        self.node = getattr(DR_init, "__dsr__node", None)
        if self.node is None:
            raise RuntimeError("DR_init.__dsr__node is not set (create the node first)")
        from dsr_msgs2.srv import MoveStop
        self._MoveStop = MoveStop
        self._stop_cli = self.node.create_client(MoveStop, MOVE_STOP_SERVICE)
        self.label = label
        self.total = total
        self.step = f"[{label}] before start (Tool/TCP and speed setup)"
        self.force_on = False
        self.compliance_on = False
        self.gripper = None

    def require_probe(self):
        if self.gripper is None:
            return
        self.gripper.wait(self.gripper.held, (True,), lambda: rclpy.spin_once(self.node, timeout_sec=.05),
                          'probe grasp check failed')

    def command_gripper(self, command):
        from DSR_ROBOT2 import wait
        command()
        wait(GRIP_WAIT)

    # log
    def info(self, msg):
        self.node.get_logger().info(msg)

    def warn(self, msg):
        self.node.get_logger().warning(msg)

    def error(self, msg):
        self.node.get_logger().error(msg)

    def begin(self, n, text, caution=None):
        if self.until is not None and n > self.until:
            raise StopAfterStep(n - 1)        # the previous step's motion has already finished (movel/movej wait until done)
        self.step = f"[{self.label} {n}/{self.total}] {text}"
        self.info(self.step)
        if caution:
            self.warn(f"    caution: {caution}")

    # cleanup
    def _move_stop(self):
        if not self._stop_cli.wait_for_service(timeout_sec=2.0):
            self.error(f"  ① move_stop service missing: {self._stop_cli.srv_name}")
            return
        req = self._MoveStop.Request()
        req.stop_mode = MOVE_STOP_MODE
        fut = self._stop_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=3.0)
        if fut.done() and fut.result() is not None and fut.result().success:
            self.warn(f"  ① move_stop(stop_mode={MOVE_STOP_MODE}, DR_QSTOP) done")
        else:
            self.error("  ① move_stop response failed/timed out")

    def cleanup(self):
        """On an error/Ctrl+C: ① stop motion ② release force ③ release compliance ④ log the stopped step. No move home."""
        self.error(f"[{self.label}] stopped → starting cleanup (no automatic move home)")
        try:
            self._move_stop()
        except BaseException as e:            # keep cleaning up even if Ctrl+C comes during cleanup
            self.error(f"  ① exception during move_stop: {e!r}")
        if self.force_on:
            try:
                from DSR_ROBOT2 import release_force
                ret = release_force(time=0.0)
                self.force_on = ret != 0
                self.warn(f"  ② release_force → ret={ret}")
            except BaseException as e:
                self.error(f"  ② exception during release_force: {e!r}")
        else:
            self.info("  ② force control off (release skipped)")
        if self.compliance_on:
            try:
                from DSR_ROBOT2 import release_compliance_ctrl
                ret = release_compliance_ctrl()
                self.compliance_on = ret != 0
                self.warn(f"  ③ release_compliance_ctrl → ret={ret}")
            except BaseException as e:
                self.error(f"  ③ exception during release_compliance_ctrl: {e!r}")
        else:
            self.info("  ③ compliance control off (release skipped)")
        self.error(f"  ④ stopped step: {self.step}")
        self.error("     Return with the pendant. Check the gripper state (holding the bonnet/probe or not) first.")

    def stop_here(self, last):
        """When stopping at --until: no stop command, only release the force/compliance that is on. No move home."""
        self.warn(f"[{self.label}] --until {self.until}: done up to step {last}, stopping here. No automatic move home")
        if self.force_on:
            from DSR_ROBOT2 import release_force
            ret = release_force(time=0.0)
            self.force_on = ret != 0
            self.warn(f"  force control was on → release_force ret={ret}")
        if self.compliance_on:
            from DSR_ROBOT2 import release_compliance_ctrl
            ret = release_compliance_ctrl()
            self.compliance_on = ret != 0
            self.warn(f"  compliance control was on → release_compliance_ctrl ret={ret}")
        self.warn(f"  last step: {self.step}")
        self.warn("  Check the gripper state (holding the bonnet/probe or not) and return with the pendant.")

    def close(self):
        try:
            self.node.destroy_client(self._stop_cli)
        except Exception:
            pass


# DSR_ROBOT2's movej/movel call wait_for_service first, but the functions below do not: they
# call_async right away and wait without a timeout → called before the service is discovered, the request is dropped and it hangs forever.
# So first wait until the DSR client objects this script uses are connected.
DSR_SERVICE_WAIT_SEC = 10.0
_DSR_CLIENTS = [                       # (client variable inside DSR_ROBOT2, functions using it)
    ("_ros2_set_singularity_handling", "set_singular_handling"),
    ("_ros2_set_digital_output", "set_digital_output"),
    ("_ros2_task_compliance_ctrl", "task_compliance_ctrl"),
    ("_ros2_set_stiffnessx", "set_stiffnessx"),
    ("_ros2_set_desired_force", "set_desired_force"),
    ("_ros2_release_force", "release_force"),
    ("_ros2_release_compliance_ctrl", "release_compliance_ctrl"),
    ("_ros2_movej", "movej"),
    ("_ros2_movel", "movel"),
]
_DSR_TOOL_CLIENTS = [                  # only without --virtual
    ("_ros2_set_current_tool", "set_tool"),
    ("_ros2_set_current_tcp", "set_tcp"),
    ("_ros2_get_current_tool", "get_tool"),
    ("_ros2_get_current_tcp", "get_tcp"),
]


def _wait_dsr_services(r, virtual):
    import DSR_ROBOT2
    clients = _DSR_CLIENTS + ([] if virtual else _DSR_TOOL_CLIENTS)
    missing = []
    for var, fn in clients:
        cli = getattr(DSR_ROBOT2, var)
        if not cli.wait_for_service(timeout_sec=DSR_SERVICE_WAIT_SEC):
            missing.append(f"{fn} ({cli.srv_name})")
    if missing:
        raise StepFailed(f"DSR services not connected ({DSR_SERVICE_WAIT_SEC:.0f}s): " + ", ".join(missing)
                         + " → exiting without moving (run the bringup / check ROS_DOMAIN_ID)")
    r.info(f"{len(clients)} DSR services connected")


def _setup(r, virtual):
    """Service connection check + Tool/TCP check + DRL lines 5-9. On a failed check, raise without moving the robot."""
    _wait_dsr_services(r, virtual)
    from DSR_ROBOT2 import (set_tool, set_tcp, get_tool, get_tcp,
                            set_singular_handling, set_velj, set_accj, set_velx, set_accx,
                            DR_AVOID)
    if virtual:
        r.warn(f"--virtual: Tool/TCP setup and check skipped (the virtual robot has no '{TOOL_NAME}' / "
               f"'{TCP_NAME}'). Do not use --virtual on the real robot.")
    else:
        r.gripper = RG2State(r.node, load('real_scene')['probe']['held_width_mm'])
        # on the real robot set_tool/set_tcp return -1 even when applied correctly
        # → do not stop on the return value; check by comparing the get_tool/get_tcp names below
        ret_tool = set_tool(TOOL_NAME)
        ret_tcp = set_tcp(TCP_NAME)
        r.info(f"set_tool('{TOOL_NAME}') → ret={ret_tool}, set_tcp('{TCP_NAME}') → ret={ret_tcp} (return values ignored)")
        tool, tcp = get_tool(), get_tcp()
        if tool != TOOL_NAME or tcp != TCP_NAME:
            raise StepFailed(f"Tool/TCP mismatch: get_tool()={tool!r} (expected {TOOL_NAME!r}), "
                             f"get_tcp()={tcp!r} (expected {TCP_NAME!r}) → exiting without moving")
        r.info(f"Tool='{tool}', TCP='{tcp}' confirmed")

    _ok(set_singular_handling(DR_AVOID), "set_singular_handling")   # DRL 5
    _ok(set_velj(VELJ), "set_velj")                                  # DRL 6  (60 × SPEED_SCALE)
    _ok(set_accj(ACCJ), "set_accj")                                  # DRL 7  (100 × SPEED_SCALE)
    _ok(set_velx(VELX[0], VELX[1]), "set_velx")                      # DRL 8  (250, 80.625 × SPEED_SCALE) DR_OFF dropped
    _ok(set_accx(ACCX[0], ACCX[1]), "set_accx")                      # DRL 9  (1000, 322.5 × SPEED_SCALE)
    r.info(f"Speed {SPEED_SCALE:.0%}: velj={VELJ}, accj={ACCJ}, velx={VELX}, accx={ACCX}")


# ---------------------------------------------------------------- disassembly
def disassemble(virtual=False, until=None, probe_docked=False):
    """m0609_bonnet.drl  (until=N: only up to step N)"""
    from DSR_ROBOT2 import (movej, movel,
                            task_compliance_ctrl, set_stiffnessx, set_desired_force,
                            release_force, release_compliance_ctrl,
                            DR_BASE, DR_MV_MOD_ABS, DR_MV_MOD_REL, DR_MV_RA_DUPLICATE,
                            DR_FC_MOD_ABS)
    from DR_common2 import posx, posj

    r = _Run("disassembly", 9, until)
    done = False
    try:
        _setup(r, virtual)

        r.begin(1, "home")
        r.require_probe()
        _ok(movej(posj(0.00, 0.00, 90.00, 0.00, 90.00, 0.00), radius=0.00, ra=DR_MV_RA_DUPLICATE), "DRL13 movej")

        if not (virtual and probe_docked):
            r.begin(2, "above the probe rack → approach downward")
            _ok(movel(posx(350.00, -224.49, 100.00, 1.94, 179.96, 90.57), radius=0.00, ref=DR_BASE, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE), "DRL15 movel")
            _ok(movel(posx(350.00, -224.49, 22.00, 1.94, 179.96, 90.57), radius=0.00, ref=DR_BASE, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE), "DRL17 movel")

            r.begin(3, "put the probe down in the rack (open the gripper) → up")
            r.require_probe()
            r.command_gripper(grip_open)                                 # DRL 19, 21
            _ok(movel(posx(349.42, -224.49, 100.00, 1.94, 179.96, 90.57), radius=0.00, ref=DR_BASE, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE), "DRL23 movel")

        r.begin(4, "via home → approach in front of the bonnet",
                caution="line 27 long linear move home → front of the bonnet (ry -136.4), the path may change to avoid a singularity")
        _ok(movej(posj(0.00, 0.00, 90.00, 0.00, 90.00, 0.00), radius=0.00, ra=DR_MV_RA_DUPLICATE), "DRL25 movej")
        _ok(movel(posx(578, 2.78, 140, 180, -136.4, 180), radius=0.00, ref=DR_BASE, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE), "DRL27 movel")
        _ok(movel(posx(598, 2.78, 140, 180, -136.4, 180), radius=0.00, ref=DR_BASE, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE), "DRL27 movel")
        _ok(movel(posx(598, 2.78, 150, 180, -136.4, 180), radius=0.00, ref=DR_BASE, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE), "DRL27 movel")

        r.begin(5, f"compliance + force 2N (+x), lift the bonnet front z+20 ({T_FORCE_MOVE:.1f} s)")
        r.compliance_on = True
        _ok(task_compliance_ctrl(), "DRL31 task_compliance_ctrl")
        _ok(set_stiffnessx([3000.00, 3000.00, 3000.00, 200.00, 200.00, 200.00], time=0.0), "DRL32 set_stiffnessx")
        r.force_on = True
        _ok(set_desired_force([2.00, 0.00, 0.00, 0.00, 0.00, 0.00], [1, 0, 0, 0, 0, 0], time=0.0, mod=DR_FC_MOD_ABS), "DRL34 set_desired_force")
        # DRL time=9.33 → 9.33 / SPEED_SCALE
        _ok(movel(posx(0.00, 0.00, 20.00, 0.00, 0.00, 0.00), time=T_FORCE_MOVE, radius=0.00, ref=DR_BASE, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE), "DRL36 movel")

        r.begin(6, f"compliance + force held, x+6 z-6 ({T_FORCE_MOVE:.1f} s) → release force/compliance",
                caution="x+6 move command along the force control axis (x, 2N). The actual travel may differ")
        # DRL time=9.33 → 9.33 / SPEED_SCALE
        _ok(movel(posx(6.00, 0.00, -6.00, 0.00, 0.00, 0.00), time=T_FORCE_MOVE, radius=0.00, ref=DR_BASE, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE), "DRL38 movel")
        _ok(release_force(time=0.0), "DRL40 release_force")
        r.force_on = False
        _ok(release_compliance_ctrl(), "DRL42 release_compliance_ctrl")
        r.compliance_on = False

        r.begin(7, "grasp the bonnet (close the gripper) → pull out")
        r.command_gripper(grip_close)                                    # DRL 44, 46
        _ok(movel(posx(-8.00, 0.00, 0.00, 0.00, 0.00, 0.00), radius=0.00, ref=DR_BASE, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE), "DRL48 movel")
        _ok(movel(posx(-5.00, 0.00, 5.00, 0.00, 0.00, 0.00), radius=0.00, ref=DR_BASE, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE), "DRL48 movel")
        _ok(movel(posx(0.00, 0.00, 20.00, 0.00, 0.00, 0.00), radius=0.00, ref=DR_BASE, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE), "DRL50 movel")

        r.begin(8, "move to the old bonnet rack → lower (z-100)",
                caution="line 52 linear move holding the bonnet with about a 30° orientation change / line 54 descent to z≈41 without force sensing")
        _ok(movel(posx(354.61, 224.58, 141.41, 159.33, 179.91, 159.21), radius=0.00, ref=DR_BASE, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE), "DRL52 movel")
        _ok(movel(posx(0.00, 0.00, -100.00, 0.00, 0.00, 0.00), radius=0.00, ref=DR_BASE, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE), "DRL54 movel")

        r.begin(9, "release the old bonnet (open the gripper) → home")
        r.command_gripper(grip_open)                                     # DRL 56, 58
        _ok(movej(posj(0.00, 0.00, 90.00, 0.00, 90.00, 0.00), radius=0.00, ra=DR_MV_RA_DUPLICATE), "DRL60 movej")

        r.info("[disassembly] done")
        done = True
    except StopAfterStep as s:
        r.stop_here(s.args[0])
        done = True
    finally:
        if not done:
            r.cleanup()
        r.close()


# ---------------------------------------------------------------- assembly
def assemble(virtual=False, until=None):
    """m0609_bonnet_assembly.drl  (until=N: only up to step N)"""
    from DSR_ROBOT2 import (movej, movel,
                            task_compliance_ctrl, set_stiffnessx, release_compliance_ctrl,
                            DR_BASE, DR_MV_MOD_ABS, DR_MV_MOD_REL, DR_MV_RA_DUPLICATE)
    from DR_common2 import posx, posj

    r = _Run("assembly", 8, until)
    done = False
    try:
        _setup(r, virtual)

        r.begin(1, "home → open the gripper")
        _ok(movej(posj(0.00, 0.00, 90.00, 0.00, 90.00, 0.00), radius=0.00, ra=DR_MV_RA_DUPLICATE), "DRL13 movej")
        # differs from the source: safety - open the gripper fully before going down to the part
        r.command_gripper(grip_open)

        r.begin(2, "above the new bonnet → approach downward",
                caution="line 17 descent to z 25.77 without force sensing")
        _ok(movel(posx(250.89, 223.00, 106.77, 90.66, 179.97, 90.51), radius=0.00, ref=DR_BASE, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE), "DRL15 movel")
        _ok(movel(posx(250.89, 223.00, 25.77, 90.66, 179.97, 90.51), radius=0.00, ref=DR_BASE, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE), "DRL17 movel")

        # rx/rz jump 90→0 only because of the ZYZ notation near ry≈180; the actual rotation is a tilt change of about 9°
        r.begin(3, "push in x+8.45 beside the part (z 25.77) while tilting about 9° to the grasp position",
                caution="line 19 linear move with tilt right beside the part")
        _ok(movel(posx(259.34, 223.00, 25.77, 0.20, 171.15, 0.04), radius=0.00, ref=DR_BASE, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE), "DRL19 movel")

        r.begin(4, "grasp the new bonnet (close the gripper)")
        r.command_gripper(grip_close)                                    # DRL 21, 23, 25

        r.begin(5, "lift z+200 → in front of the car → mounting pose",
                caution="line 31 linear move holding the bonnet with about a 30° tilt in front of the car")
        _ok(movel(posx(0.00, 0.00, 200.00, 0.00, 0.00, 0.00), radius=0.00, ref=DR_BASE, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE), "DRL27 movel")
        _ok(movel(posx(600.34, 0.00, 206.77, 90.55, 179.97, 90.40), radius=0.00, ref=DR_BASE, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE), "DRL29 movel")
        _ok(movel(posx(604.54, -0.01, 182.00, 0.06, 150.15, -0.11), radius=0.00, ref=DR_BASE, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE), "DRL31 movel")

        r.begin(6, "compliance, x+10 z+1 → ry-20 → release compliance")
        r.compliance_on = True
        _ok(task_compliance_ctrl(), "DRL33 task_compliance_ctrl")
        _ok(set_stiffnessx([3000.00, 3000.00, 3000.00, 200.00, 200.00, 200.00], time=0.0), "DRL34 set_stiffnessx")
        _ok(movel(posx(10.00, 0.00, 1.00, 0.00, 0.00, 0.00), radius=0.00, ref=DR_BASE, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE), "DRL36 movel")
        _ok(movel(posx(0.00, 0.00, 0.00, 0.00, -20.00, 0.00), radius=0.00, ref=DR_BASE, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE), "DRL38 movel")
        _ok(release_compliance_ctrl(), "DRL40 release_compliance_ctrl")
        r.compliance_on = False

        r.begin(7, "release the new bonnet (open the gripper) → back off x-20 z-20",
                caution="line 46 relative move z -20 (downward) right after opening the gripper")
        r.command_gripper(grip_open)                                     # DRL 42, 44
        _ok(movel(posx(-20.00, 0.00, -20.00, 0.00, 0.00, 0.00), radius=0.00, ref=DR_BASE, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE), "DRL46 movel")

        r.begin(8, "bonnet front pose → back off → up z+50 → home",
                caution="ends without picking the probe up again (as in the source)")
        _ok(movel(posx(573.13, 10.26, 129.30, 179.76, -150.86, 178.81), radius=0.00, ref=DR_BASE, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE), "DRL48 movel")
        _ok(movel(posx(593.13, 10.26, 129.30, 179.76, -150.86, 178.81), radius=0.00, ref=DR_BASE, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE), "DRL50 movel")
        # differs from the source: safety - no movej right from the bonnet gap (+20mm); retreat linearly to the line 48 pose, then up
        _ok(movel(posx(573.13, 10.26, 129.30, 179.76, -150.86, 178.81), radius=0.00, ref=DR_BASE, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE), "added movel (retreat to the line 48 pose)")
        _ok(movel(posx(0.00, 0.00, 50.00, 0.00, 0.00, 0.00), radius=0.00, ref=DR_BASE, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE), "added movel (z+50)")
        _ok(movej(posj(0.00, 0.00, 90.00, 0.00, 90.00, 0.00), radius=0.00, ra=DR_MV_RA_DUPLICATE), "DRL52 movej")

        r.info("[assembly] done")
        done = True
    except StopAfterStep as s:
        r.stop_here(s.args[0])
        done = True
    finally:
        if not done:
            r.cleanup()
        r.close()


# ---------------------------------------------------------------- main
def main(argv=None):
    parser = argparse.ArgumentParser(description="Bonnet disassembly and assembly (DRL port)")
    parser.add_argument("mode", choices=["disassembly", "assembly", "all"])
    parser.add_argument("--virtual", action="store_true",
                        help="virtual robot: skip the Tool/TCP setup and check")
    parser.add_argument("--probe-docked", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--until", type=int, metavar="N",
                        help="stop after step N (disassembly 1-9 / assembly 1-8, cannot be used with all)")
    args = parser.parse_args(remove_ros_args()[1:] if argv is None else argv)
    if args.probe_docked and (not args.virtual or args.mode != "disassembly" or args.until is not None):
        parser.error("--probe-docked is only supported for a full virtual bonnet disassembly run")
    if args.until is not None:
        if args.mode == "all":
            parser.error("--until can only be used with one of disassembly or assembly")
        total = 9 if args.mode == "disassembly" else 8
        if not 1 <= args.until <= total:
            parser.error(f"--until must be 1..{total}")

    # keep rclpy from catching Ctrl+C → KeyboardInterrupt runs the finally cleanup
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("bonnet_replace", namespace=ROBOT_ID)
    DR_init.__dsr__id = node.get_namespace().strip('/') or ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL
    setattr(DR_init, "__dsr__node", node)

    rc = 0
    try:
        if args.mode in ("disassembly", "all"):
            disassemble(virtual=args.virtual, until=args.until, probe_docked=args.probe_docked)
        if args.mode in ("assembly", "all"):
            assemble(virtual=args.virtual, until=args.until)
        node.get_logger().info(f"=== {args.mode} done ===")
    except KeyboardInterrupt:
        node.get_logger().error("Stopped by Ctrl+C")
        rc = 130
    except Exception as e:
        node.get_logger().error(f"Stopped by an error: {e!r}")
        rc = 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
