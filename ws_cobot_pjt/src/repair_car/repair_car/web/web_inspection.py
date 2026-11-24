"""Ordered web inspection steps, each with its own plan, process and report."""
from collections import deque
from dataclasses import dataclass
import math
import time

from repair_car.web.web_process import ManagedProcess


PARTS = {'bonnet': ('bonnet', ''), 'left_door': ('door', 'left'), 'right_door': ('door', 'right')}
PART_LABELS = {'bonnet': '보닛', 'left_door': '왼쪽 도어', 'right_door': '오른쪽 도어'}


@dataclass
class InspectionStep:
    part: str
    region: int
    arguments: list[str]


def inspection_regions(value):
    if not isinstance(value, list):
        raise ValueError('탐색 영역 목록이 필요합니다')
    for box in value:
        if (not isinstance(box, list) or len(box) != 4 or
                not all(type(v) in (int, float) and math.isfinite(v) for v in box) or
                box[2] <= box[0] or box[3] <= box[1]):
            raise ValueError('부분 탐색은 유효한 직사각형 좌표가 필요합니다')
    return value


class InspectionProcess:
    """A managed process slot that stays occupied through placement changes and all scans. A scan ended by the probe's
    force guard (ContactProbe.guard, 'Force exceeded') runs again, up to RETRIES times per step: the new run lifts straight up
    before HOME (RobotProgram.home), off the surface it stopped on."""
    RETRIES = 3
    FORCE_STOP = 'Force exceeded'

    def __init__(self, steps, pose_check, require_ready, env=None, *, pickup=None):
        self.pending = deque(steps)
        self.total, self.index = len(steps), 0
        self.pose_check = pose_check
        self.require_ready, self.env = require_ready, env
        self.pickup = pickup
        self.step = self.child = self.returncode = None
        self.stop_due, self.killed, self.started_at = 0., False, time.time()
        self.phase, self.error, self.history = '', '', ''
        self.retries = 0
        self.advance()

    def advance(self):
        previous = self.step.part if self.step else None
        if not self.pending:
            self.returncode = 0
            return
        self.step = self.pending.popleft()
        self.index, self.retries = self.index + 1, 0
        if self.pose_check(previous, self.step.part):
            self.phase = 'waiting_pose'
        else:
            self.begin_step()

    def begin_step(self):
        if self.pickup is not None:
            arguments, check_empty = self.pickup
            check_empty()
            self.child = ManagedProcess(arguments, env=self.env)
            self.pickup = None
            self.phase = 'preparing_probe'
            return
        if self.require_ready is not None:
            self.require_ready()
        self.child = ManagedProcess(self.step.arguments, env=self.env)
        self.phase = 'scanning'

    def confirm(self):
        if self.phase != 'waiting_pose' or self.stop_due:
            raise ValueError('배치 확인을 기다리는 작업이 없습니다')
        self.begin_step()

    def poll(self):
        if self.child is None:
            return self.returncode
        code = self.child.poll()
        self.killed |= self.child.killed
        if code is None:
            return None
        preparing = self.phase == 'preparing_probe'
        if code and not preparing and not self.stop_due and self.retries < self.RETRIES and self.FORCE_STOP in self.child.log(1500):
            self.retries += 1
            self.history = (self.history + self.child.log() + f'\n{self.FORCE_STOP}로 종료: 같은 단계를 다시 실행합니다 '
                                                              f'({self.retries}/{self.RETRIES})\n')[-6000:]
            self.child.close()
            self.child = None
            try:
                self.begin_step()
            except (ValueError, OSError, RuntimeError) as error:
                self.error, self.returncode = str(error), 1
            return self.returncode
        if code and not self.stop_due:
            lines = self.child.log(2000).strip().splitlines()
            reason = lines[-1].removeprefix('RuntimeError: ') if lines else f'작업 종료 (코드 {code})'
            region = f' 영역 {self.step.region}' if self.step.region else ''
            self.error = f'탐침 파지 · {reason}' if preparing else f'{PART_LABELS[self.step.part]}{region} · {reason}'
        if self.stop_due or code:
            return code
        self.history = (self.history + self.child.log() + '\n')[-6000:]
        self.child.close()
        self.child = None
        try:
            if preparing:
                self.begin_step()
            else:
                self.advance()
        except (ValueError, OSError, RuntimeError) as error:
            self.error, self.returncode = str(error), 1
        return self.returncode

    def interrupt(self, grace):
        self.stop_due = time.monotonic() + grace
        if self.child is not None:
            self.child.interrupt(grace)
        else:
            self.returncode = 1

    def log(self):
        return (self.history + (self.child.log() if self.child else ''))[-6000:]

    def close(self):
        if self.child is not None:
            self.child.close()

    def status(self):
        return dict(step=self.index, total=self.total, part=self.step.part, label=PART_LABELS[self.step.part],
                    region=self.step.region, phase=self.phase, needs_pickup=self.pickup is not None)
