#!/usr/bin/env python3
"""보닛 분해·조립 (펜던트 DRL → DSR_ROBOT2 파이썬 이식)

원본:
  drl/m0609_bonnet.drl          → disassemble()
  drl/m0609_bonnet_assembly.drl → assemble()

실행:
  python3 bonnet_replace.py disassembly | assembly | all [--virtual]

다른 코드에서 쓸 때 (예: replace_parts):
  호출하는 쪽이 DR_init.__dsr__id / __dsr__model / __dsr__node 를 먼저 설정한 뒤
  from bonnet_replace import disassemble, assemble
  disassemble(); assemble()
  * 실패·중단 시 정리(동작 정지 → 힘 해제 → 순응 해제 → 단계 로그) 후 예외를 다시 던짐.
  * set_velj/accj/velx/accx 는 DSR_ROBOT2 전역값이라 호출 뒤 다른 동작의 기본 속도도 바뀜.

이식 규칙:
  - 좌표, 순서, ref(0 = DR_BASE), mod, 강성, 힘, 해제 순서는 DRL 그대로.
  - while gLoop < 1 → 한 번 실행.
  - 설치된 DSR_ROBOT2 와 다른 점:
      * movel 에 app_type 인자가 없음 → DRL 의 app_type=DR_MV_APP_NONE(접근 동작 없음) 은 뺌.
      * set_velx 는 인자 2개만 받음, DR_OFF 상수도 없음 → DRL 의 3번째 인자 DR_OFF 는 뺌.
  - 모든 DSR 명령의 반환값(-1 = 실패)을 확인해서, 실패하면 다음 줄로 가지 않고 멈춤.
  - 그리퍼 동작 뒤 wait(1.0) 추가 (조립 DRL 25줄의 원래 wait(1.00) 은 그대로 사용).
  - "원본과 다름: 안전" 주석이 붙은 곳은 원본 DRL 과 다르게 고친 곳.
  - 에러/Ctrl+C 시 자동 홈 복귀는 하지 않음 (복귀는 펜던트로).
"""
import argparse
import sys

import rclpy
from rclpy.signals import SignalHandlerOptions
import DR_init

# ---------------------------------------------------------------- 설정
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

# DR_init 속성은 문자열로 접근 (클래스 안에서 __dsr__xxx 를 쓰면 이름 맹글링으로 _Cls__dsr__xxx 가 됨)
setattr(DR_init, "__dsr__id", ROBOT_ID)
setattr(DR_init, "__dsr__model", ROBOT_MODEL)

TOOL_NAME = "Tool Weight v2"
TCP_NAME = "GripperDA_v1_probe"

SPEED_SCALE = 0.2                                        # DRL 기본 속도 대비 비율
VELJ = 60.0 * SPEED_SCALE                                # 12.0
ACCJ = 100.0 * SPEED_SCALE                               # 20.0
VELX = (250.0 * SPEED_SCALE, 80.625 * SPEED_SCALE)       # (50.0, 16.125)
ACCX = (1000.0 * SPEED_SCALE, 322.5 * SPEED_SCALE)       # (200.0, 64.5)
# time= 으로 정한 movel 은 시간을 SPEED_SCALE 로 나눠서 같은 비율로 느리게
T_FORCE_MOVE = 9.33 / SPEED_SCALE                        # 46.65 [s] (분해 DRL 36, 38줄)

GRIP_WAIT = 1.0                                          # 그리퍼 동작 뒤 대기 [s]

MOVE_STOP_SERVICE = f"/{ROBOT_ID}/dsr_controller2/motion/move_stop"
MOVE_STOP_MODE = 1                                       # DR_QSTOP (Stop Category 2)


class StepFailed(RuntimeError):
    """DSR 명령이 실패(-1)했거나 시작 조건이 맞지 않음."""


class StopAfterStep(Exception):
    """--until N: N 단계까지 끝내고 일부러 멈춤 (에러 아님)."""


def _ok(ret, what):
    if ret != 0:
        raise StepFailed(f"{what} 실패 (ret={ret})")


# ---------------------------------------------------------------- 그리퍼
def grip_close():
    """DO1 ON, DO2 OFF (대기는 부르는 쪽에서)."""
    from DSR_ROBOT2 import set_digital_output, ON, OFF
    _ok(set_digital_output(2, OFF), "set_digital_output(2, OFF)")
    _ok(set_digital_output(1, ON), "set_digital_output(1, ON)")


def grip_open():
    """DO1 OFF, DO2 ON (대기는 부르는 쪽에서)."""
    from DSR_ROBOT2 import set_digital_output, ON, OFF
    _ok(set_digital_output(1, OFF), "set_digital_output(1, OFF)")
    _ok(set_digital_output(2, ON), "set_digital_output(2, ON)")


# ---------------------------------------------------------------- 실행 상태 / 정리
class _Run:
    """단계 기록, 힘·순응 상태, 실패 시 정리."""

    def __init__(self, label, total, until=None):
        self.until = until                    # None 이면 끝까지, N 이면 N 단계까지 하고 멈춤
        self.node = getattr(DR_init, "__dsr__node", None)
        if self.node is None:
            raise RuntimeError("DR_init.__dsr__node 가 설정되지 않음 (노드를 먼저 만들어야 함)")
        from dsr_msgs2.srv import MoveStop
        self._MoveStop = MoveStop
        self._stop_cli = self.node.create_client(MoveStop, MOVE_STOP_SERVICE)
        self.label = label
        self.total = total
        self.step = f"[{label}] 시작 전 (Tool/TCP·속도 설정)"
        self.force_on = False
        self.compliance_on = False

    # 로그
    def info(self, msg):
        self.node.get_logger().info(msg)

    def warn(self, msg):
        self.node.get_logger().warning(msg)

    def error(self, msg):
        self.node.get_logger().error(msg)

    def z(self):
        try:
            import DSR_ROBOT2
            from DSR_ROBOT2 import get_current_posx, DR_BASE
            if not DSR_ROBOT2._ros2_get_current_posx.service_is_ready():
                return "?"                    # 연결 전에 부르면 영원히 멈추므로 생략
            pos, _ = get_current_posx(ref=DR_BASE)
            return f"{pos[2]:.2f}"
        except Exception:
            return "?"

    def begin(self, n, text, caution=None):
        if self.until is not None and n > self.until:
            raise StopAfterStep(n - 1)        # 이전 단계 동작은 이미 끝난 상태 (movel/movej 는 끝날 때까지 기다림)
        self.step = f"[{self.label} {n}/{self.total}] {text}"
        self.info(f"{self.step}  (현재 z={self.z()})")
        if caution:
            self.warn(f"    주의: {caution}")

    # 정리
    def _move_stop(self):
        if not self._stop_cli.wait_for_service(timeout_sec=2.0):
            self.error(f"  ① move_stop 서비스 없음: {MOVE_STOP_SERVICE}")
            return
        req = self._MoveStop.Request()
        req.stop_mode = MOVE_STOP_MODE
        fut = self._stop_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=3.0)
        if fut.done() and fut.result() is not None and fut.result().success:
            self.warn(f"  ① move_stop(stop_mode={MOVE_STOP_MODE}, DR_QSTOP) 완료")
        else:
            self.error("  ① move_stop 응답 실패/시간 초과")

    def cleanup(self):
        """에러·Ctrl+C 시: ① 동작 정지 ② 힘 해제 ③ 순응 해제 ④ 멈춘 단계 로그. 홈 이동 없음."""
        self.error(f"[{self.label}] 중단됨 → 정리 시작 (자동 홈 이동 없음)")
        try:
            self._move_stop()
        except BaseException as e:            # 정리 중 Ctrl+C 가 와도 다음 정리는 계속
            self.error(f"  ① move_stop 중 예외: {e!r}")
        if self.force_on:
            try:
                from DSR_ROBOT2 import release_force
                ret = release_force(time=0.0)
                self.force_on = ret != 0
                self.warn(f"  ② release_force → ret={ret}")
            except BaseException as e:
                self.error(f"  ② release_force 중 예외: {e!r}")
        else:
            self.info("  ② 힘 제어 꺼져 있음 (해제 생략)")
        if self.compliance_on:
            try:
                from DSR_ROBOT2 import release_compliance_ctrl
                ret = release_compliance_ctrl()
                self.compliance_on = ret != 0
                self.warn(f"  ③ release_compliance_ctrl → ret={ret}")
            except BaseException as e:
                self.error(f"  ③ release_compliance_ctrl 중 예외: {e!r}")
        else:
            self.info("  ③ 순응 제어 꺼져 있음 (해제 생략)")
        self.error(f"  ④ 멈춘 단계: {self.step}  (현재 z={self.z()})")
        self.error("     복귀는 펜던트로 하세요. 그리퍼 상태(보닛/탐침 파지 여부)를 먼저 확인하세요.")

    def stop_here(self, last):
        """--until 으로 멈출 때: 동작 정지 명령 없이, 켜져 있는 힘·순응만 해제. 홈 이동 없음."""
        self.warn(f"[{self.label}] --until {self.until}: {last}단계까지 완료, 여기서 멈춤 "
                  f"(현재 z={self.z()}). 자동 홈 이동 없음")
        if self.force_on:
            from DSR_ROBOT2 import release_force
            ret = release_force(time=0.0)
            self.force_on = ret != 0
            self.warn(f"  힘 제어 켜져 있었음 → release_force ret={ret}")
        if self.compliance_on:
            from DSR_ROBOT2 import release_compliance_ctrl
            ret = release_compliance_ctrl()
            self.compliance_on = ret != 0
            self.warn(f"  순응 제어 켜져 있었음 → release_compliance_ctrl ret={ret}")
        self.warn(f"  마지막 단계: {self.step}")
        self.warn("  그리퍼 상태(보닛/탐침 파지 여부)를 확인하고, 복귀는 펜던트로 하세요.")

    def close(self):
        try:
            self.node.destroy_client(self._stop_cli)
        except Exception:
            pass


# DSR_ROBOT2 의 movej/movel 은 호출 전에 wait_for_service 를 하지만, 아래 함수들은 하지 않고
# 바로 call_async 후 시간 제한 없이 기다림 → 서비스 연결(discovery) 전에 부르면 요청이 버려지고 영원히 멈춤.
# 그래서 이 스크립트가 쓰는 DSR 클라이언트 객체가 연결될 때까지 먼저 기다림.
DSR_SERVICE_WAIT_SEC = 10.0
_DSR_CLIENTS = [                       # (DSR_ROBOT2 안의 클라이언트 변수, 쓰는 함수)
    ("_ros2_set_singularity_handling", "set_singular_handling"),
    ("_ros2_get_current_posx", "get_current_posx"),
    ("_ros2_set_digital_output", "set_digital_output"),
    ("_ros2_task_compliance_ctrl", "task_compliance_ctrl"),
    ("_ros2_set_stiffnessx", "set_stiffnessx"),
    ("_ros2_set_desired_force", "set_desired_force"),
    ("_ros2_release_force", "release_force"),
    ("_ros2_release_compliance_ctrl", "release_compliance_ctrl"),
    ("_ros2_movej", "movej"),
    ("_ros2_movel", "movel"),
]
_DSR_TOOL_CLIENTS = [                  # --virtual 이 아닐 때만
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
            missing.append(f"{fn} ({cli.service_name})")
    if missing:
        raise StepFailed(f"DSR 서비스 연결 안 됨 ({DSR_SERVICE_WAIT_SEC:.0f}s): " + ", ".join(missing)
                         + " → 움직이지 않고 종료 (bringup 실행 / ROS_DOMAIN_ID 확인)")
    r.info(f"DSR 서비스 {len(clients)}개 연결 확인")


def _setup(r, virtual):
    """서비스 연결 확인 + Tool/TCP 확인 + DRL 5~9줄. 확인 실패 시 로봇을 움직이지 않고 예외."""
    _wait_dsr_services(r, virtual)
    from DSR_ROBOT2 import (set_tool, set_tcp, get_tool, get_tcp,
                            set_singular_handling, set_velj, set_accj, set_velx, set_accx,
                            DR_AVOID)
    if virtual:
        r.warn(f"--virtual: Tool/TCP 설정·확인 건너뜀 (가상 로봇에는 '{TOOL_NAME}' / "
               f"'{TCP_NAME}' 없음). 실물에서는 --virtual 쓰지 마세요.")
    else:
        # 실물에서 set_tool/set_tcp 는 제대로 적용돼도 -1 을 돌려줌
        # → 반환값으로 멈추지 않고, 아래 get_tool/get_tcp 이름 비교로 확인
        ret_tool = set_tool(TOOL_NAME)
        ret_tcp = set_tcp(TCP_NAME)
        r.info(f"set_tool('{TOOL_NAME}') → ret={ret_tool}, set_tcp('{TCP_NAME}') → ret={ret_tcp} (반환값은 무시)")
        tool, tcp = get_tool(), get_tcp()
        if tool != TOOL_NAME or tcp != TCP_NAME:
            raise StepFailed(f"Tool/TCP 불일치: get_tool()={tool!r} (기대 {TOOL_NAME!r}), "
                             f"get_tcp()={tcp!r} (기대 {TCP_NAME!r}) → 움직이지 않고 종료")
        r.info(f"Tool='{tool}', TCP='{tcp}' 확인")

    _ok(set_singular_handling(DR_AVOID), "set_singular_handling")   # DRL 5
    _ok(set_velj(VELJ), "set_velj")                                  # DRL 6  (60 × SPEED_SCALE)
    _ok(set_accj(ACCJ), "set_accj")                                  # DRL 7  (100 × SPEED_SCALE)
    _ok(set_velx(VELX[0], VELX[1]), "set_velx")                      # DRL 8  (250, 80.625 × SPEED_SCALE) DR_OFF 뺌
    _ok(set_accx(ACCX[0], ACCX[1]), "set_accx")                      # DRL 9  (1000, 322.5 × SPEED_SCALE)
    r.info(f"속도 {SPEED_SCALE:.0%}: velj={VELJ}, accj={ACCJ}, velx={VELX}, accx={ACCX}")


# ---------------------------------------------------------------- 분해
def disassemble(virtual=False, until=None):
    """m0609_bonnet.drl  (until=N: N 단계까지만)"""
    from DSR_ROBOT2 import (movej, movel, wait,
                            task_compliance_ctrl, set_stiffnessx, set_desired_force,
                            release_force, release_compliance_ctrl,
                            DR_BASE, DR_MV_MOD_ABS, DR_MV_MOD_REL, DR_MV_RA_DUPLICATE,
                            DR_FC_MOD_ABS)
    from DR_common2 import posx, posj

    r = _Run("분해", 9, until)
    done = False
    try:
        _setup(r, virtual)

        r.begin(1, "홈")
        _ok(movej(posj(0.00, 0.00, 90.00, 0.00, 90.00, 0.00), radius=0.00, ra=DR_MV_RA_DUPLICATE), "DRL13 movej")

        r.begin(2, "탐침 거치대 위 → 아래로 접근")
        _ok(movel(posx(350.00, -224.49, 100.00, 1.94, 179.96, 90.57), radius=0.00, ref=DR_BASE, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE), "DRL15 movel")
        _ok(movel(posx(350.00, -224.49, 22.00, 1.94, 179.96, 90.57), radius=0.00, ref=DR_BASE, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE), "DRL17 movel")

        r.begin(3, "탐침 거치대에 내려놓기 (그리퍼 열기) → 위로")
        grip_open()                                                     # DRL 19, 21
        wait(GRIP_WAIT)                                                 # 추가: 그리퍼 대기
        _ok(movel(posx(349.42, -224.49, 100.00, 1.94, 179.96, 90.57), radius=0.00, ref=DR_BASE, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE), "DRL23 movel")

        r.begin(4, "홈 경유 → 보닛 앞 접근",
                caution="27줄 홈→보닛 앞 큰 직선 이동(ry -150), 특이점 회피로 경로가 달라질 수 있음")
        _ok(movej(posj(0.00, 0.00, 90.00, 0.00, 90.00, 0.00), radius=0.00, ra=DR_MV_RA_DUPLICATE), "DRL25 movej")
        _ok(movel(posx(572.25, 2.85, 148.70, 178.60, -150.33, 178.23), radius=0.00, ref=DR_BASE, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE), "DRL27 movel")
        _ok(movel(posx(587.25, 2.85, 138.70, 178.60, -150.33, 178.23), radius=0.00, ref=DR_BASE, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE), "DRL27 movel")
        _ok(movel(posx(593.25, 2.85, 129.70, 178.60, -150.33, 178.23), radius=0.00, ref=DR_BASE, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE), "DRL29 movel")

        r.begin(5, f"순응 + 힘 3N(+x), 보닛 앞 들기 z+40 ({T_FORCE_MOVE:.1f}초)")
        r.compliance_on = True
        _ok(task_compliance_ctrl(), "DRL31 task_compliance_ctrl")
        _ok(set_stiffnessx([3000.00, 3000.00, 3000.00, 200.00, 200.00, 200.00], time=0.0), "DRL32 set_stiffnessx")
        r.force_on = True
        _ok(set_desired_force([3.00, 0.00, 0.00, 0.00, 0.00, 0.00], [1, 0, 0, 0, 0, 0], time=0.0, mod=DR_FC_MOD_ABS), "DRL34 set_desired_force")
        # DRL time=9.33 → 9.33 / SPEED_SCALE
        _ok(movel(posx(0.00, 0.00, 40.00, 0.00, 0.00, 0.00), time=T_FORCE_MOVE, radius=0.00, ref=DR_BASE, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE), "DRL36 movel")

        r.begin(6, f"순응 + 힘 유지, x+10 z-10 ({T_FORCE_MOVE:.1f}초) → 힘·순응 해제",
                caution="힘 제어 축(x, 3N)과 같은 방향으로 x+10 이동 명령. 실제 이동량이 다를 수 있음")
        # DRL time=9.33 → 9.33 / SPEED_SCALE
        _ok(movel(posx(10.00, 0.00, -10.00, 0.00, 0.00, 0.00), time=T_FORCE_MOVE, radius=0.00, ref=DR_BASE, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE), "DRL38 movel")
        _ok(release_force(time=0.0), "DRL40 release_force")
        r.force_on = False
        _ok(release_compliance_ctrl(), "DRL42 release_compliance_ctrl")
        r.compliance_on = False

        r.begin(7, "보닛 잡기 (그리퍼 닫기) → 빼내기")
        grip_close()                                                    # DRL 44, 46
        wait(GRIP_WAIT)                                                 # 추가: 그리퍼 대기
        _ok(movel(posx(-8.00, 0.00, 0.00, 0.00, 0.00, 0.00), radius=0.00, ref=DR_BASE, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE), "DRL48 movel")
        _ok(movel(posx(-5.00, 0.00, 5.00, 0.00, 0.00, 0.00), radius=0.00, ref=DR_BASE, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE), "DRL48 movel")
        _ok(movel(posx(0.00, 0.00, 20.00, 0.00, 0.00, 0.00), radius=0.00, ref=DR_BASE, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE), "DRL50 movel")

        r.begin(8, "헌 보닛 거치대로 이동 → 내리기 (z-100)",
                caution="52줄 보닛 든 채 자세가 크게 바뀌는 직선 이동 / 54줄 힘 감지 없이 z≈41까지 하강")
        _ok(movel(posx(354.61, 224.58, 141.41, 159.33, 179.91, 159.21), radius=0.00, ref=DR_BASE, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE), "DRL52 movel")
        _ok(movel(posx(0.00, 0.00, -100.00, 0.00, 0.00, 0.00), radius=0.00, ref=DR_BASE, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE), "DRL54 movel")

        r.begin(9, "헌 보닛 놓기 (그리퍼 열기) → 홈")
        grip_open()                                                     # DRL 56, 58
        wait(GRIP_WAIT)                                                 # 추가: 그리퍼 대기
        _ok(movej(posj(0.00, 0.00, 90.00, 0.00, 90.00, 0.00), radius=0.00, ra=DR_MV_RA_DUPLICATE), "DRL60 movej")

        r.info(f"[분해] 완료 (현재 z={r.z()})")
        done = True
    except StopAfterStep as s:
        r.stop_here(s.args[0])
        done = True
    finally:
        if not done:
            r.cleanup()
        r.close()


# ---------------------------------------------------------------- 조립
def assemble(virtual=False, until=None):
    """m0609_bonnet_assembly.drl  (until=N: N 단계까지만)"""
    from DSR_ROBOT2 import (movej, movel, wait,
                            task_compliance_ctrl, set_stiffnessx, release_compliance_ctrl,
                            DR_BASE, DR_MV_MOD_ABS, DR_MV_MOD_REL, DR_MV_RA_DUPLICATE)
    from DR_common2 import posx, posj

    r = _Run("조립", 8, until)
    done = False
    try:
        _setup(r, virtual)

        r.begin(1, "홈 → 그리퍼 열기")
        _ok(movej(posj(0.00, 0.00, 90.00, 0.00, 90.00, 0.00), radius=0.00, ra=DR_MV_RA_DUPLICATE), "DRL13 movej")
        # 원본과 다름: 안전 — 부품으로 내려가기 전에 그리퍼를 확실히 열어 둠
        grip_open()
        wait(GRIP_WAIT)

        r.begin(2, "새 보닛 위 → 아래로 접근",
                caution="17줄 힘 감지 없이 z 25.77까지 하강")
        _ok(movel(posx(250.89, 223.00, 106.77, 90.66, 179.97, 90.51), radius=0.00, ref=DR_BASE, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE), "DRL15 movel")
        _ok(movel(posx(250.89, 223.00, 25.77, 90.66, 179.97, 90.51), radius=0.00, ref=DR_BASE, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE), "DRL17 movel")

        r.begin(3, "부품 옆(z 25.77)에서 rz 90→0 회전하며 파지 위치로",
                caution="19줄 부품 바로 옆에서 큰 회전 + 직선 이동")
        _ok(movel(posx(259.34, 223.00, 25.77, 0.20, 171.15, 0.04), radius=0.00, ref=DR_BASE, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE), "DRL19 movel")

        r.begin(4, "새 보닛 잡기 (그리퍼 닫기)")
        grip_close()                                                    # DRL 21, 23
        wait(1.00)                                                      # DRL 25 (원본 wait)

        r.begin(5, "들어올림 z+200 → 차 앞 → 장착 자세",
                caution="31줄 보닛 든 채 차 앞에서 rx/ry/rz 동시에 크게 바뀌는 직선 이동")
        _ok(movel(posx(0.00, 0.00, 200.00, 0.00, 0.00, 0.00), radius=0.00, ref=DR_BASE, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE), "DRL27 movel")
        _ok(movel(posx(600.34, 0.00, 206.77, 90.55, 179.97, 90.40), radius=0.00, ref=DR_BASE, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE), "DRL29 movel")
        _ok(movel(posx(604.54, -0.01, 182.00, 0.06, 150.15, -0.11), radius=0.00, ref=DR_BASE, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE), "DRL31 movel")

        r.begin(6, "순응, x+10 → ry-10 → 순응 해제")
        r.compliance_on = True
        _ok(task_compliance_ctrl(), "DRL33 task_compliance_ctrl")
        _ok(set_stiffnessx([3000.00, 3000.00, 3000.00, 200.00, 200.00, 200.00], time=0.0), "DRL34 set_stiffnessx")
        _ok(movel(posx(10.00, 0.00, 1.00, 0.00, 0.00, 0.00), radius=0.00, ref=DR_BASE, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE), "DRL36 movel")
        _ok(movel(posx(0.00, 0.00, 0.00, 0.00, -20.00, 0.00), radius=0.00, ref=DR_BASE, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE), "DRL38 movel")
        _ok(release_compliance_ctrl(), "DRL40 release_compliance_ctrl")
        r.compliance_on = False

        r.begin(7, "새 보닛 놓기 (그리퍼 열기) → 빠지기 x-20 z-20",
                caution="46줄 그리퍼 연 직후 상대 이동 z -20 (아래로)")
        grip_open()                                                     # DRL 42, 44
        wait(GRIP_WAIT)                                                 # 추가: 그리퍼 대기
        _ok(movel(posx(-20.00, 0.00, -20.00, 0.00, 0.00, 0.00), radius=0.00, ref=DR_BASE, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE), "DRL46 movel")

        r.begin(8, "보닛 앞 자세 → 빠지기 → 위로 z+50 → 홈",
                caution="탐침을 다시 집지 않고 끝남 (원본 그대로)")
        _ok(movel(posx(573.13, 10.26, 129.30, 179.76, -150.86, 178.81), radius=0.00, ref=DR_BASE, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE), "DRL48 movel")
        _ok(movel(posx(593.13, 10.26, 129.30, 179.76, -150.86, 178.81), radius=0.00, ref=DR_BASE, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE), "DRL50 movel")
        # 원본과 다름: 안전 — 보닛 틈(+20mm)에서 바로 movej 하지 않고 48줄 자세로 직선 후퇴 후 위로
        _ok(movel(posx(573.13, 10.26, 129.30, 179.76, -150.86, 178.81), radius=0.00, ref=DR_BASE, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE), "추가 movel (48줄 자세로 후퇴)")
        _ok(movel(posx(0.00, 0.00, 50.00, 0.00, 0.00, 0.00), radius=0.00, ref=DR_BASE, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE), "추가 movel (z+50)")
        _ok(movej(posj(0.00, 0.00, 90.00, 0.00, 90.00, 0.00), radius=0.00, ra=DR_MV_RA_DUPLICATE), "DRL52 movej")

        r.info(f"[조립] 완료 (현재 z={r.z()})")
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
    parser = argparse.ArgumentParser(description="보닛 분해·조립 (DRL 이식)")
    parser.add_argument("mode", choices=["disassembly", "assembly", "all"])
    parser.add_argument("--virtual", action="store_true",
                        help="가상 로봇: Tool/TCP 설정·확인 건너뜀")
    parser.add_argument("--until", type=int, metavar="N",
                        help="N 단계까지만 하고 멈춤 (disassembly 1~9 / assembly 1~8, all 과 같이 못 씀)")
    args = parser.parse_args(argv)
    if args.until is not None:
        if args.mode == "all":
            parser.error("--until 은 disassembly 또는 assembly 하나에만 쓸 수 있음")
        total = 9 if args.mode == "disassembly" else 8
        if not 1 <= args.until <= total:
            parser.error(f"--until 은 1~{total}")

    # Ctrl+C 를 rclpy 가 가로채지 않게 → KeyboardInterrupt 로 finally 정리 코드가 돎
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("bonnet_replace", namespace=ROBOT_ID)
    setattr(DR_init, "__dsr__node", node)

    rc = 0
    try:
        if args.mode in ("disassembly", "all"):
            disassemble(virtual=args.virtual, until=args.until)
        if args.mode in ("assembly", "all"):
            assemble(virtual=args.virtual, until=args.until)
        node.get_logger().info(f"=== {args.mode} 완료 ===")
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


if __name__ == "__main__":
    sys.exit(main())
