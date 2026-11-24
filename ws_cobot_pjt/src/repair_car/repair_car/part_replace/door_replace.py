#!/usr/bin/env python3
"""Door disassembly and assembly (pendant DRL ported to DSR_ROBOT2 Python)

Source:
  drl/m0609_door.drl   lines 13-49 → disassemble_left()
  drl/m0609_door2.drl          → assemble_left()
  The right door (no taught source in this repository): the left door program mirrored about the BASE XZ plane
  (y -> -y, ZYZ a, c -> -a, -c). With the right door toward the robot (vehicle yaw -90) the cell is the mirror image of
  the left door one (yaw 90) about BASE y = 0; the door racks mirror with it. Verified in the virtual demo_final only.

Run:
  ros2 run repair_car door_replace <disassembly|assembly|all> [--side left|right] [--virtual] [--until N]

Use from other code:
  set DR_init.__dsr__id / __dsr__model / __dsr__node first, then
  from repair_car.part_replace.door_replace import disassemble_left, assemble_left (or disassemble, assemble with side)

Porting rules as in bonnet_replace.py (Tool/TCP check, SPEED_SCALE speeds, return value checks, cleanup on failure, no automatic return home).
  - movel with explicit vel=/acc= in the DRL: the linear speed is already slow (1, 10 mm/s) and kept,
    the rotation speed and acceleration are multiplied by SPEED_SCALE.
  - wait(GRIP_WAIT) added after gripper actions (the assembly DRL's own wait(2.00), wait(0.50) kept).
  - Places commented "differs from the source: safety" deviate from the source DRL.
"""
import argparse
import sys

import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args
import DR_init

if __package__:
    from .bonnet_replace import (ROBOT_ID, ROBOT_MODEL, SPEED_SCALE,
                                 StopAfterStep, _ok, _Run, _setup, grip_close, grip_open)
else:
    from bonnet_replace import (ROBOT_ID, ROBOT_MODEL, SPEED_SCALE,
                                StopAfterStep, _ok, _Run, _setup, grip_close, grip_open)

HOME = (0.00, 0.00, 90.00, 0.00, 90.00, 0.00)


def _vel(linear, angular):
    """DRL vel=[linear, rotation]: linear kept, rotation times SPEED_SCALE."""
    return [linear, angular * SPEED_SCALE]


def _acc(linear, angular):
    """DRL acc=[linear, rotation]: both times SPEED_SCALE."""
    return [linear * SPEED_SCALE, angular * SPEED_SCALE]


def _side(side):
    """BASE pose (x, y, z, a, b, c) / relative move -> the one for `side` (right: mirrored about BASE y = 0)."""
    if side == "left":
        return lambda p: p
    return lambda p: (p[0], -p[1], p[2], -p[3], p[4], -p[5])


def _run(label, total, until, virtual, body):
    """Same frame as bonnet_replace: setup → run the steps → (stop at --until | cleanup on failure)."""
    r = _Run(label, total, until)
    done = False
    try:
        _setup(r, virtual)
        body(r)
        r.info(f"[{label}] done")
        done = True
    except StopAfterStep as s:
        r.stop_here(s.args[0])
        done = True
    finally:
        if not done:
            r.cleanup()
        r.close()


# ---------------------------------------------------------------- disassembly: left door
def disassemble(virtual=False, until=None, side="left"):
    """m0609_door.drl lines 13-49  (until=N: only up to step N)"""
    from DSR_ROBOT2 import movej, movel, DR_BASE, DR_MV_MOD_ABS, DR_MV_MOD_REL, DR_MV_RA_DUPLICATE
    from DR_common2 import posx, posj
    pose = _side(side)

    def L(p, what, mod=DR_MV_MOD_ABS):
        _ok(movel(posx(*pose(p)), radius=0.00, ref=DR_BASE, mod=mod, ra=DR_MV_RA_DUPLICATE), what)

    def body(r):
        r.begin(1, "home")
        _ok(movej(posj(*HOME), radius=0.00, ra=DR_MV_RA_DUPLICATE), "DRL13 movej")

        r.begin(2, f"approach in front of the {side} door → x+18 → z-10",
                caution="lines 15→17 change the orientation by about 20° in a long linear move toward the car")
        L((367.64, 7.55, 142.07, 14.16, 136.33, 13.44), "DRL15 movel")
        L((621.18, 15.83, 132.22, 10.92, 116.93, 8.09), "DRL17 movel")
        L((639.16, 15.84, 132.26, 10.92, 116.93, 8.09), "DRL19 movel")
        L((639.13, 15.84, 122.19, 10.92, 116.94, 8.10), "DRL21 movel")

        r.begin(3, "release the door latch: x-6 → x-1 z+8 → x-30",
                caution="pushes the door out without opening or closing the gripper, as started (as in the source)")
        L((-6.00, 0.00, 0.00, 0.00, 0.00, 0.00), "DRL23 movel", DR_MV_MOD_REL)
        L((-1.00, 0.00, 8.00, 0.00, 0.00, 0.00), "DRL25 movel", DR_MV_MOD_REL)
        L((-30.00, 0.00, 0.00, 0.00, 0.00, 0.00), "DRL27 movel", DR_MV_MOD_REL)

        r.begin(4, "move aside → door grasp orientation → y+30",
                caution="lines 29→31 linear move beside the door with a large orientation change (rz 8→83°)")
        L((599.22, -56.35, 128.48, 10.92, 116.94, 8.10), "DRL29 movel")
        L((609.00, 9.69, 120.48, 36.62, 124.66, 83.25), "DRL31 movel")
        L((609.00, 39.67, 120.48, 36.62, 124.66, 90.00), "DRL33 movel")

        r.begin(5, f"grasp the {side} door (close the gripper) → z+10 → x-30 y-10")
        r.command_gripper(grip_close)                                    # DRL 35, 37
        L((0.00, 0.00, 10.00, 0.00, 0.00, 0.00), "DRL39 movel", DR_MV_MOD_REL)
        L((-30.00, -10.00, 0.00, 0.00, 0.00, 0.00), "DRL41 movel", DR_MV_MOD_REL)

        r.begin(6, "move to the old door rack → lower (z-80)",
                caution="line 43 long linear move holding the door / line 45 descent to z≈46.6 without force sensing")
        L((344.81, 264.48, 126.56, 79.14, 130.58, 84.02), "DRL43 movel")
        L((0.00, 0.00, -80.00, 0.00, 0.00, 0.00), "DRL45 movel", DR_MV_MOD_REL)

        r.begin(7, "release the old door (open the gripper)")
        r.command_gripper(grip_open)                                     # DRL 47, 49

    _run("disassembly", 7, until, virtual, body)


def disassemble_left(virtual=False, until=None):
    disassemble(virtual, until, "left")


# ---------------------------------------------------------------- assembly: left door
def assemble(virtual=False, until=None, side="left"):
    """m0609_door2.drl  (until=N: only up to step N)"""
    from DSR_ROBOT2 import (movej, movel, wait, task_compliance_ctrl, set_stiffnessx, release_compliance_ctrl,
                            DR_BASE, DR_MV_MOD_ABS, DR_MV_MOD_REL, DR_MV_RA_DUPLICATE)
    from DR_common2 import posx, posj
    pose = _side(side)

    def L(p, what, mod=DR_MV_MOD_ABS, **kw):
        _ok(movel(posx(*pose(p)), radius=0.00, ref=DR_BASE, mod=mod, ra=DR_MV_RA_DUPLICATE, **kw), what)

    def body(r):
        # differs from the source: safety - the source does the line 13 movel right from the current position. Start at home and open the gripper before going down to the door
        r.begin(1, "home → open the gripper")
        _ok(movej(posj(*HOME), radius=0.00, ra=DR_MV_RA_DUPLICATE), "added movej (home)")
        r.command_gripper(grip_open)

        r.begin(2, "above the new door → approach downward", caution="line 15 descent to z 2.08 without force sensing")
        L((445.16, 194.18, 59.28, 95.67, 142.28, 94.10), "DRL13 movel")
        L((440.65, 239.16, 2.08, 96.00, 142.59, 94.56), "DRL15 movel")

        r.begin(3, "grasp the new door (close the gripper)")
        grip_close()                                                    # DRL 17, 19
        wait(2.00)                                                      # DRL 21 (source wait)

        r.begin(4, "lift z 20 → mounting pose beside the car → z-10",
                caution="line 25 long linear move toward the car holding the door with a large orientation change")
        L((440.65, 239.16, 20.00, 96.00, 142.59, 94.56), "DRL23 movel")
        L((578.74, 30.00, 140.00, 179.21, -142.44, -80.03), "DRL25 movel")
        L((578.74, 30.00, 130.00, 179.21, -142.44, -80.03), "DRL27 movel")

        r.begin(5, "compliance (stiffness 200), insert x+1 z-15 at 1 mm/s → release compliance")
        r.compliance_on = True
        _ok(task_compliance_ctrl(), "DRL29 task_compliance_ctrl")
        _ok(set_stiffnessx([200.00, 200.00, 200.00, 200.00, 200.00, 200.00], time=0.0), "DRL30 set_stiffnessx")
        L((1.00, 0.00, -15.00, 0.00, 0.00, 0.00), "DRL32 movel", DR_MV_MOD_REL,
          vel=_vel(1.00, 60.08), acc=_acc(1000.00, 240.33))
        _ok(release_compliance_ctrl(), "DRL34 release_compliance_ctrl")
        r.compliance_on = False

        r.begin(6, "release the new door (open the gripper) → move up")
        grip_open()                                                     # DRL 36, 38
        wait(0.50)                                                      # DRL 40 (source wait)
        L((548.11, 28.05, 161.60, 178.97, -142.22, -80.21), "DRL42 movel")

        r.begin(7, "close the gripper → push the door shut (last move 10 mm/s)",
                caution="pushes the door with the closed gripper. The source ends here (no return home)")
        r.command_gripper(grip_close)                                    # DRL 44, 46
        L((572.22, 58.19, 103.18, 177.57, -142.01, -81.18), "DRL48 movel")
        L((621.09, -1.05, 108.68, 175.29, -141.93, -82.34), "DRL50 movel",
          vel=_vel(10.00, 60.83), acc=_acc(1000.00, 243.30))

    _run("assembly", 7, until, virtual, body)


def assemble_left(virtual=False, until=None):
    assemble(virtual, until, "left")


# ---------------------------------------------------------------- main
def run_main(name, choices, until_help, argv=None):
    """Shared main. choices: {mode: [(function, step count), ...]}"""
    parser = argparse.ArgumentParser(description=f"{name} (DRL port)")
    parser.add_argument("mode", choices=list(choices))
    parser.add_argument("--virtual", action="store_true", help="virtual robot: skip the Tool/TCP setup and check")
    parser.add_argument("--side", choices=["left", "right"], default="left", help="door (right: the left program mirrored)")
    parser.add_argument("--until", type=int, metavar="N", help=until_help)
    args = parser.parse_args(remove_ros_args()[1:] if argv is None else argv)
    steps = choices[args.mode]
    if args.until is not None:
        if len(steps) != 1:
            parser.error("--until cannot be used with all")
        if not 1 <= args.until <= steps[0][1]:
            parser.error(f"--until must be 1..{steps[0][1]}")

    # keep rclpy from catching Ctrl+C → KeyboardInterrupt runs the finally cleanup
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node(name.replace(" ", "_"), namespace=ROBOT_ID)
    DR_init.__dsr__id = node.get_namespace().strip('/') or ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL
    setattr(DR_init, "__dsr__node", node)

    rc = 0
    try:
        for fn, _ in steps:
            fn(virtual=args.virtual, until=args.until, side=args.side)
        node.get_logger().info(f"=== {name} {args.mode} done ===")
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


def main(argv=None):
    return run_main("door_replace", {
        "disassembly": [(disassemble, 7)],
        "assembly": [(assemble, 7)],
        "all": [(disassemble, 7), (assemble, 7)],
    }, "stop after step N (disassembly 1-7 / assembly 1-7, cannot be used with all)", argv)


if __name__ == "__main__":
    sys.exit(main())
