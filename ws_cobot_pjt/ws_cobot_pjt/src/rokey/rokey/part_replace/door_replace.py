#!/usr/bin/env python3
"""왼쪽 문 분해·조립 (펜던트 DRL → DSR_ROBOT2 파이썬 이식)

원본:
  drl/m0609_door.drl   13~49줄 → disassemble_left()
  drl/m0609_door2.drl          → assemble_left()
  (오른쪽 문 분해 51~81줄은 아직 이식 안 함)

실행:
  ros2 run rokey door_replace <disassembly|assembly|all> [--virtual] [--until N]

다른 코드에서 쓸 때:
  DR_init.__dsr__id / __dsr__model / __dsr__node 를 먼저 설정한 뒤
  from rokey.part_replace.door_replace import disassemble_left, assemble_left

이식 규칙은 bonnet_replace.py 와 같음 (Tool/TCP 확인, 속도 SPEED_SCALE, 반환값 확인, 실패 시 정리, 자동 홈 복귀 없음).
  - DRL 에서 vel=/acc= 를 직접 준 movel: 직선 속도는 이미 느려서(1, 10 mm/s) 그대로,
    회전 속도와 가속도는 SPEED_SCALE 을 곱함.
  - 그리퍼 동작 뒤 wait(GRIP_WAIT) 추가 (조립 DRL 의 원래 wait(2.00), wait(0.50) 은 그대로).
  - "원본과 다름: 안전" 주석이 붙은 곳은 원본 DRL 과 다르게 고친 곳.
"""
import argparse
import sys

import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args
import DR_init

if __package__:
    from .bonnet_replace import (ROBOT_ID, ROBOT_MODEL, SPEED_SCALE, GRIP_WAIT,
                                 StopAfterStep, _ok, _Run, _setup, grip_close, grip_open)
else:
    from bonnet_replace import (ROBOT_ID, ROBOT_MODEL, SPEED_SCALE, GRIP_WAIT,
                                StopAfterStep, _ok, _Run, _setup, grip_close, grip_open)

HOME = (0.00, 0.00, 90.00, 0.00, 90.00, 0.00)


def _vel(linear, angular):
    """DRL 의 vel=[직선, 회전]: 직선은 그대로, 회전은 SPEED_SCALE 배."""
    return [linear, angular * SPEED_SCALE]


def _acc(linear, angular):
    """DRL 의 acc=[직선, 회전]: 둘 다 SPEED_SCALE 배."""
    return [linear * SPEED_SCALE, angular * SPEED_SCALE]


def _run(label, total, until, virtual, body):
    """bonnet_replace 와 같은 틀: 설정 → 단계 실행 → (--until 멈춤 | 실패 시 정리)."""
    r = _Run(label, total, until)
    done = False
    try:
        _setup(r, virtual)
        body(r)
        r.info(f"[{label}] 완료 (현재 z={r.z()})")
        done = True
    except StopAfterStep as s:
        r.stop_here(s.args[0])
        done = True
    finally:
        if not done:
            r.cleanup()
        r.close()


# ---------------------------------------------------------------- 분해: 왼쪽 문
def disassemble_left(virtual=False, until=None):
    """m0609_door.drl 13~49줄  (until=N: N 단계까지만)"""
    from DSR_ROBOT2 import movej, movel, wait, DR_BASE, DR_MV_MOD_ABS, DR_MV_MOD_REL, DR_MV_RA_DUPLICATE
    from DR_common2 import posx, posj

    def L(p, what, mod=DR_MV_MOD_ABS):
        _ok(movel(posx(*p), radius=0.00, ref=DR_BASE, mod=mod, ra=DR_MV_RA_DUPLICATE), what)

    def body(r):
        r.begin(1, "홈")
        _ok(movej(posj(*HOME), radius=0.00, ra=DR_MV_RA_DUPLICATE), "DRL13 movej")

        r.begin(2, "왼쪽 문 앞 접근 → x+18 → z-10",
                caution="15→17줄 자세를 약 20° 바꾸며 차 쪽으로 큰 직선 이동")
        L((367.64, 7.55, 142.07, 14.16, 136.33, 13.44), "DRL15 movel")
        L((621.18, 15.83, 132.22, 10.92, 116.93, 8.09), "DRL17 movel")
        L((639.16, 15.84, 132.26, 10.92, 116.93, 8.09), "DRL19 movel")
        L((639.13, 15.84, 122.19, 10.92, 116.94, 8.10), "DRL21 movel")

        r.begin(3, "문 걸쇠 풀기: x-6 → x-1 z+8 → x-30",
                caution="그리퍼를 열거나 닫지 않고 시작한 상태 그대로 문을 밀어 냄 (원본 그대로)")
        L((-6.00, 0.00, 0.00, 0.00, 0.00, 0.00), "DRL23 movel", DR_MV_MOD_REL)
        L((-1.00, 0.00, 8.00, 0.00, 0.00, 0.00), "DRL25 movel", DR_MV_MOD_REL)
        L((-30.00, 0.00, 0.00, 0.00, 0.00, 0.00), "DRL27 movel", DR_MV_MOD_REL)

        r.begin(4, "옆으로 빠짐 → 문 잡는 자세로 → y+30",
                caution="29→31줄 문 옆에서 자세를 크게(rz 8→83°) 바꾸는 직선 이동")
        L((599.22, -56.35, 128.48, 10.92, 116.94, 8.10), "DRL29 movel")
        L((609.00, 9.69, 120.48, 36.62, 124.66, 83.25), "DRL31 movel")
        L((609.00, 39.67, 120.48, 36.62, 124.66, 90.00), "DRL33 movel")

        r.begin(5, "왼쪽 문 잡기 (그리퍼 닫기) → z+10 → x-30 y-10")
        grip_close()                                                    # DRL 35, 37
        wait(GRIP_WAIT)                                                 # 추가: 그리퍼 대기
        L((0.00, 0.00, 10.00, 0.00, 0.00, 0.00), "DRL39 movel", DR_MV_MOD_REL)
        L((-30.00, -10.00, 0.00, 0.00, 0.00, 0.00), "DRL41 movel", DR_MV_MOD_REL)

        r.begin(6, "헌 문 거치대로 이동 → 내리기 (z-80)",
                caution="43줄 문을 든 채 큰 직선 이동 / 45줄 힘 감지 없이 z≈46.6까지 하강")
        L((344.81, 264.48, 126.56, 79.14, 130.58, 84.02), "DRL43 movel")
        L((0.00, 0.00, -80.00, 0.00, 0.00, 0.00), "DRL45 movel", DR_MV_MOD_REL)

        r.begin(7, "헌 문 놓기 (그리퍼 열기)")
        grip_open()                                                     # DRL 47, 49
        wait(GRIP_WAIT)                                                 # 추가: 그리퍼 대기

    _run("왼문 분해", 7, until, virtual, body)


# ---------------------------------------------------------------- 조립: 왼쪽 문
def assemble_left(virtual=False, until=None):
    """m0609_door2.drl  (until=N: N 단계까지만)"""
    from DSR_ROBOT2 import (movej, movel, wait, task_compliance_ctrl, set_stiffnessx, release_compliance_ctrl,
                            DR_BASE, DR_MV_MOD_ABS, DR_MV_MOD_REL, DR_MV_RA_DUPLICATE)
    from DR_common2 import posx, posj

    def L(p, what, mod=DR_MV_MOD_ABS, **kw):
        _ok(movel(posx(*p), radius=0.00, ref=DR_BASE, mod=mod, ra=DR_MV_RA_DUPLICATE, **kw), what)

    def body(r):
        # 원본과 다름: 안전 — 원본은 현재 위치에서 바로 13줄 movel. 홈에서 시작하고, 문으로 내려가기 전에 그리퍼를 엶
        r.begin(1, "홈 → 그리퍼 열기")
        _ok(movej(posj(*HOME), radius=0.00, ra=DR_MV_RA_DUPLICATE), "추가 movej (홈)")
        grip_open()
        wait(GRIP_WAIT)

        r.begin(2, "새 문 위 → 아래로 접근", caution="15줄 힘 감지 없이 z 2.08까지 하강")
        L((445.16, 194.18, 59.28, 95.67, 142.28, 94.10), "DRL13 movel")
        L((440.65, 239.16, 2.08, 96.00, 142.59, 94.56), "DRL15 movel")

        r.begin(3, "새 문 잡기 (그리퍼 닫기)")
        grip_close()                                                    # DRL 17, 19
        wait(2.00)                                                      # DRL 21 (원본 wait)

        r.begin(4, "들어올림 z 20 → 차 옆 장착 자세 → z-10",
                caution="25줄 문을 든 채 자세를 크게 바꾸며 차 쪽으로 큰 직선 이동")
        L((440.65, 239.16, 20.00, 96.00, 142.59, 94.56), "DRL23 movel")
        L((578.74, 30.00, 140.00, 179.21, -142.44, -80.03), "DRL25 movel")
        L((578.74, 30.00, 130.00, 179.21, -142.44, -80.03), "DRL27 movel")

        r.begin(5, "순응(강성 200), x+1 z-15 를 1 mm/s 로 끼우기 → 순응 해제")
        r.compliance_on = True
        _ok(task_compliance_ctrl(), "DRL29 task_compliance_ctrl")
        _ok(set_stiffnessx([200.00, 200.00, 200.00, 200.00, 200.00, 200.00], time=0.0), "DRL30 set_stiffnessx")
        L((1.00, 0.00, -15.00, 0.00, 0.00, 0.00), "DRL32 movel", DR_MV_MOD_REL,
          vel=_vel(1.00, 60.08), acc=_acc(1000.00, 240.33))
        _ok(release_compliance_ctrl(), "DRL34 release_compliance_ctrl")
        r.compliance_on = False

        r.begin(6, "새 문 놓기 (그리퍼 열기) → 위로 빠짐")
        grip_open()                                                     # DRL 36, 38
        wait(0.50)                                                      # DRL 40 (원본 wait)
        L((548.11, 28.05, 161.60, 178.97, -142.22, -80.21), "DRL42 movel")

        r.begin(7, "그리퍼 닫기 → 문 밀어 닫기 (마지막 이동 10 mm/s)",
                caution="닫은 그리퍼로 문을 밀어 붙임. 원본은 여기서 끝남 (홈 복귀 없음)")
        grip_close()                                                    # DRL 44, 46
        wait(GRIP_WAIT)                                                 # 추가: 그리퍼 대기
        L((572.22, 58.19, 103.18, 177.57, -142.01, -81.18), "DRL48 movel")
        L((621.09, -1.05, 108.68, 175.29, -141.93, -82.34), "DRL50 movel",
          vel=_vel(10.00, 60.83), acc=_acc(1000.00, 243.30))

    _run("왼문 조립", 7, until, virtual, body)


# ---------------------------------------------------------------- main
def run_main(name, choices, until_help, argv=None):
    """공통 main. choices: {mode: [(함수, 단계 수), ...]}"""
    parser = argparse.ArgumentParser(description=f"{name} (DRL 이식)")
    parser.add_argument("mode", choices=list(choices))
    parser.add_argument("--virtual", action="store_true", help="가상 로봇: Tool/TCP 설정·확인 건너뜀")
    parser.add_argument("--until", type=int, metavar="N", help=until_help)
    args = parser.parse_args(remove_ros_args()[1:] if argv is None else argv)
    steps = choices[args.mode]
    if args.until is not None:
        if len(steps) != 1:
            parser.error("--until 은 all 과 같이 못 씀")
        if not 1 <= args.until <= steps[0][1]:
            parser.error(f"--until 은 1~{steps[0][1]}")

    # Ctrl+C 를 rclpy 가 가로채지 않게 → KeyboardInterrupt 로 finally 정리 코드가 돎
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node(name.replace(" ", "_"), namespace=ROBOT_ID)
    DR_init.__dsr__id = node.get_namespace().strip('/') or ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL
    setattr(DR_init, "__dsr__node", node)

    rc = 0
    try:
        for fn, _ in steps:
            fn(virtual=args.virtual, until=args.until)
        node.get_logger().info(f"=== {name} {args.mode} 완료 ===")
    except KeyboardInterrupt:
        node.get_logger().error("Ctrl+C 로 중단됨")
        rc = 130
    except Exception as e:
        node.get_logger().error(f"에러로 중단됨: {e!r}")
        rc = 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return rc


def main(argv=None):
    return run_main("door_replace", {
        "disassembly": [(disassemble_left, 7)],
        "assembly": [(assemble_left, 7)],
        "all": [(disassemble_left, 7), (assemble_left, 7)],
    }, "N 단계까지만 하고 멈춤 (분해 1~7 / 조립 1~7, all 과 같이 못 씀)", argv)


if __name__ == "__main__":
    sys.exit(main())
