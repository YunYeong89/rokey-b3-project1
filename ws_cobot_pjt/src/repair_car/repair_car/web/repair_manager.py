"""Repair process node (INTERFACE.md section 5). Waits for /repair/command and runs one job at a time,
recording every stage in the repair DB:

  calibrate  probe pickup when needed (real robot: confirmed automatic job), calibration when there is no result for this mode
             (or --calibrate), the vehicle at its fixed pose (choose_car.fixed: no identification touches). A manual
             job of taught parts only (replace_parts.TAUGHT: the bonnet) skips calibration and the vehicle pose:
             those parts are replaced at pendant-taught positions.
  plan       auto mode, real vehicle: real_scan's mesh path plan (its cache or a new plan), read from its output
  scan       auto mode. The real vehicle (real robot, or a virtual session of the real assembly): real_scan
             (--controller mesh-height, as the web part inspection) per REAL_SCAN_PARTS. Completed real scan ROI
             reports drive replacement for confirmed dent/crack. Candidates alone are recorded as suspect.
             The scaled model vehicles (virtual sedan/suv/sports): scan_surface over the replaceable parts.
             No damage -> no_damage
  stock      check_db for the parts to replace (auto: dent/missing, manual: the requested parts) -> on_hold
  replace    replace_parts per part -> inspect -> replacement record, consume on success -> done / failed

Every step (probe, calibration, scans, replacements, HOME) that fails is logged and started again after a return
HOME, on the real robot too (RepairManager.retry); only a web stop ends a job. Missing stock is a warning.
Virtual demo_final (run_virtual.sh, the lift turntable stand), auto mode: for the bonnet, then the left and the right
door: probe grasp check -> HOME -> turntable and lift via the stand's LiftCommand service (and /fixture/select to the
exact demo placement) -> real_scan mesh-height -> the taught replacement (door_fl / door_fr: door_replace --side). A
step that fails is logged and started again (after a return HOME) until it succeeds; only a web stop ends the job.

A stop request ends the job with `stopped`.
  ros2 run repair_car repair_manager [--calibrate]          (or ros2 launch repair_car repair.launch.py)
"""
import argparse
import json
import os
import re
import sys
import threading
import time
from pathlib import Path
from tempfile import TemporaryDirectory

from custom_interfaces.srv import LiftCommand
from std_msgs.msg import String

if __package__:
    from repair_car.calibration.calibration import calibrate
    from repair_car.web.check_db import StockChecker
    from repair_car.inspection.choose_car import fixed
    from repair_car.config import load, read_session
    from repair_car.robot_control.robot_bridge import robot_topic
    from repair_car.calibration.force_data import CalibrationRun
    from repair_car.probe.pick_probe import ProbePickup
    from repair_car.inspection.part_scan.archive import InspectionArchive, ScanFiles
    from repair_car.probe.real_probe_pickup import RealProbePickup
    from repair_car.web.repair_db import RepairDB
    from repair_car.part_replace.replace_parts import TAUGHT, PartReplacer, ReplaceAborted
    from repair_car.robot_control.rg2_state import RG2State
    from repair_car.inspection.scan_scene import ScanScene
    from repair_car.inspection.scan_surface import scan_parts
else:
    from repair_car.calibration.calibration import calibrate
    from repair_car.web.check_db import StockChecker
    from repair_car.inspection.choose_car import fixed
    from config import load, read_session
    from robot_control.robot_bridge import robot_topic
    from calibration.force_data import CalibrationRun
    from repair_car.probe.pick_probe import ProbePickup
    from repair_car.inspection.part_scan.archive import InspectionArchive, ScanFiles
    from repair_car.probe.real_probe_pickup import RealProbePickup
    from repair_car.web.repair_db import RepairDB
    from part_replace.replace_parts import TAUGHT, PartReplacer, ReplaceAborted
    from robot_control.rg2_state import RG2State
    from repair_car.inspection.scan_scene import ScanScene
    from repair_car.inspection.scan_surface import scan_parts


PLAN_LINE = re.compile(r'mesh plan (cache hit|checked and cached): ([\d.]+) s')  # mesh_scan.run_mesh_scan
REAL_SCAN_PARTS = ('bonnet',)
# virtual demo_final: (panel, real_scan part, door side, DB part code, turntable heading deg)
DEMO_PANELS = (('bonnet', 'bonnet', None, 'bonnet', 0.), ('left_door', 'door', 'left', 'door_fl', 90.),
               ('right_door', 'door', 'right', 'door_fr', 270.))  # real vehicle: parts scanned in auto mode (only the bonnet can be replaced so far)
CLOSE_TIMEOUT_S = 5.0  # --job-id: exit anyway when the ROS shutdown hangs (the job is already recorded)


class Stopped(Exception):
    """The job ended early with a final status (no_damage, on_hold)."""

    def __init__(self, status, summary):
        super().__init__(summary)
        self.status = status


class RepairManager:
    def __init__(self, always_calibrate, front_confirmed=False):
        self.program = CalibrationRun('repair_manager')
        if self.program.mode == 'real':
            self.gripper = RG2State(self.program.node, load('real_scene')['probe']['held_width_mm'])
        self.db = RepairDB(**self.program.config['db'])
        self.always_calibrate, self.pending, self.job, self.stage = always_calibrate, None, None, None
        self.program.node.create_subscription(String, robot_topic(self.program.node, '/repair/command'), self.command, 10)
        self.program.log(f'repair_manager ready ({self.program.mode}); waiting for /repair/command')

    def command(self, message):
        """Runs inside the robot executor: only queue a start or flag a stop."""
        data = json.loads(message.data)
        if data.get('cmd') == 'start' and self.job is None:
            self.pending = data
        elif data.get('cmd') == 'stop' and self.job is not None and data.get('job_id') == self.job['job_id']:
            self.program.robot.cancelled = True
            self.program.log(f'job {self.job["job_id"]}: stop requested')

    def spin(self):
        while True:
            self.program.robot.executor.spin_once(timeout_sec=0.1)
            if self.pending is not None:
                self.job, self.pending = self.pending, None
                self.run(self.job)
                self.job = None

    def event(self, stage, message, level='info'):
        self.stage = stage
        self.program.log(f'[{stage}] {message}')
        self.db.event(self.job['job_id'], stage, message, level)

    def run(self, job):
        program, job_id = self.program, job['job_id']
        self.stage = 'prepare'
        try:
            program.reload()
            status, summary = self.process(job)
            self.db.finish(job_id, status, summary)
            return 0 if status in ('done', 'no_damage', 'on_hold') else 1
        except Stopped as end:
            self.db.finish(job_id, end.status, str(end))
            return 0
        except KeyboardInterrupt:
            program.robot.cancelled = True
            program.robot.stop()
            program.probe.close()
            self.db.finish(job_id, 'stopped', '웹에서 작업 중지')
            return 130
        except Exception as error:
            if program.robot.cancelled:
                program.probe.close()
                program.log(f'job {job_id}: stopped')
                self.db.finish(job_id, 'stopped', '웹에서 작업 중지')
            else:
                program.node.get_logger().error(f'job {job_id} failed in {self.stage}: {error}')
                try:
                    self.event(self.stage, str(error), 'error')
                    self.db.finish(job_id, 'failed', str(error))
                finally:
                    if program.mode == 'virtual' and not getattr(error, 'no_home', False):
                        program.home()
            return 1
        finally:
            program.robot.cancelled = False

    def process(self, job):
        program, job_id = self.program, job['job_id']
        manual = job.get('mode') == 'manual'
        parts = self.db.requested_parts(job_id) if manual else None
        real = self.real_vehicle()
        physical = program.mode == 'real'
        if not manual and not physical and real and ScanScene.multi_panel():
            return self.process_demo(job)
        if physical:
            if manual and parts != ['bonnet']:
                raise RuntimeError('실기 수동 교체는 보닛 한 부품을 지원합니다.')
        elif real and ScanScene.virtual_panel() != 'bonnet' and (not manual or 'bonnet' in (parts or ())):
            if not ScanScene.multi_panel():
                raise Stopped('on_hold', '보닛 수리는 --car demo_front 정면 배치에서 실행하세요.')
        if manual and parts and all(part in TAUGHT for part in parts):
            self.event('calibrate', f'job {job_id} (manual {parts}): 티칭 좌표 교체라 보정·차량 인식 생략, 탐침 확인')
            self.retry('calibrate', '탐침 파지 확인', self.hold_probe)   # the bonnet disassembly docks the held probe first
            if not physical and real:
                self.retry('calibrate', '보닛 배치', lambda: ScanScene.prepare_virtual(program, 'bonnet'))
        elif not manual and real:
            self.event('calibrate', f'job {job_id} (auto, 실차): 실측 배치(real_scene.yaml)로 스캔, 보정·차량 인식 생략')
            self.retry('calibrate', '탐침 파지 확인', self.hold_probe)
            if not physical:
                self.retry('calibrate', '보닛 배치', lambda: ScanScene.prepare_virtual(program, 'bonnet'))
        else:
            self.event('calibrate', f'job {job_id} ({job.get("mode", "auto")}): probe, calibration, fixed vehicle pose')
            self.retry('calibrate', '탐침 파지 확인', self.hold_probe)
            result = read_session('calibration_result.json')
            if self.always_calibrate or not result or result['mode'] != program.mode:
                self.retry('calibrate', '보정', lambda: calibrate(program))
            vehicle = self.retry('calibrate', '차량 위치', lambda: fixed(program))
            self.event('calibrate', f'vehicle {vehicle["type"]} at {[round(v, 1) for v in vehicle["pose_base_mm_deg"]]} '
                                    f'(고정 위치, {vehicle["source"]})')
        if not manual and real:
            findings, parts = self.retry('scan', '실차 스캔', self.scan_real)
            if findings:
                self.db.findings(job_id, findings)
            else:
                raise Stopped('no_damage', '손상 없음')
            if not parts:
                raise Stopped('done', '교체 대상 없음 (의심 판정만 있음)')
        elif not manual:
            self.event('scan', f'parts {program.config["inspection"]["replaceable"]}')
            scan = self.retry('scan', '표면 스캔', lambda: scan_parts(program, program.config['inspection']['replaceable']))
            self.event('detect', f'findings {scan["findings"]}')
            if scan['findings']:
                self.db.findings(job_id, scan['findings'])
            else:
                raise Stopped('no_damage', '손상 없음')
            parts = scan['replace']
            if not parts:
                raise Stopped('done', '교체 대상 없음 (의심 판정만 있음)')
        self.event('stock', f'parts {parts}')
        available, missing = StockChecker(self.db).check(parts)
        if missing:
            self.event('stock', f'재고 없음: {", ".join(missing)} — 교환을 계속합니다', 'warn')
        replacer, failed = PartReplacer(program, self.event), []
        if physical:
            self.retry('replace', '실기 탐침 확인', lambda: self.check_real_probe(pick=True))
        for part in parts:
            self.event('replace', part)
            outcome = self.retry('replace', f'{part}: 교환', lambda: replacer.replace(part))
            self.event('inspect', f'{part}: flush {outcome["flush_mm"]} mm')
            if outcome['flush_mm'] is not None:
                program.publish(type='flush', mm=outcome['flush_mm'])
            self.db.replacement(job_id, part, **outcome)
            if outcome['success']:
                try:
                    self.db.consume(part)
                except RuntimeError as error:
                    self.event('stock', f'{part}: 재고 차감 실패 ({error})', 'warn')
            else:
                failed.append(part)
        if not physical:
            self.retry('done', 'HOME 이동', program.home)
        self.event('done', f'replaced {[part for part in parts if part not in failed]}')
        return ('failed', f'교체 실패: {", ".join(failed)}') if failed else ('done', f'교체 완료: {", ".join(parts)}')

    def real_vehicle(self):
        """The team's real vehicle: the real robot, or a virtual session of the real assembly (--car real / demo)."""
        truth = read_session('scene_truth.json') if self.program.mode == 'virtual' else None
        return self.program.mode == 'real' or (truth or {}).get('model') == 'real'

    def check_real_probe(self, *, pick=False):
        """Confirm the real probe and acquire it when starting a repair with empty jaws."""
        program, robot = self.program, self.program.robot
        settings = load('real_scene')['probe']
        pickup = RealProbePickup(robot, settings, program.config['robot']['home_joints_deg'],
                                 gripper=self.gripper, verify_position=False)
        pickup.ensure(allow_pickup=pick,
                      report=(lambda message: self.event('calibrate', message)) if pick else None)

    @staticmethod
    def scan_findings(runs):
        """Aggregate a completed survey and its optional rescan, using measured ROI on the physical robot."""
        confirmed, candidates = set(), set()
        kinds = {'dent', 'crack'}
        for files in runs:
            result = json.loads(files.path('result.json').read_text())
            if not result.get('completed'):
                raise RuntimeError(f'완료되지 않은 검사 결과: {files.run}')
            roi = json.loads(files.path('roi.json').read_text())
            confirmed.update(region['type'] for region in roi['regions'] if region.get('confirmed'))
            candidates.update(region['type'] for region in roi['candidates'])
        return sorted(confirmed & kinds), sorted((candidates & kinds) - confirmed)

    def scan_real(self):
        """Run the whole scan and read only the reports emitted by this child."""
        findings, replace = [], []
        for part in REAL_SCAN_PARTS:
            part_findings, confirmed = self.scan_part(part)
            findings += part_findings
            if confirmed:
                replace.append(part)
        return findings, replace

    def scan_part(self, part, side=None, code=None):
        """real_scan mesh-height of one part (a door with its side) -> (DB findings, confirmed damage)."""
        program = self.program
        job = self.db.job(self.job['job_id'])
        vehicle = InspectionArchive.vehicle(self.db.vehicle(job['vehicle_id']))
        env = InspectionArchive.environment(vehicle)
        physical = program.mode == 'real'
        ros_args = ['--ros-args', '-r', f'__ns:={program.node.get_namespace()}']
        name, code = f'{part}_{side}' if side else part, code or part
        findings = []
        self.event('plan', f'{name}: 경로 계획 (real_scan, mesh-height)')
        planned = []

        def line(text):
            plan = PLAN_LINE.search(text)
            if not plan:
                return
            how = '캐시 사용' if plan[1] == 'cache hit' else '새로 계산'
            if planned:
                self.event('scan', f'{name}: 재스캔 경로 {how} ({float(plan[2]):.1f} s)')
            else:
                self.event('plan', f'{name}: 경로 계획 완료 — {how} ({float(plan[2]):.1f} s)')
                self.event('scan', f'{name}: 실차 스캔 시작')
            planned.append(plan[1])

        with TemporaryDirectory(prefix='repair_car-repair-') as pending:
            manifest = Path(pending) / 'reports.json'
            options = ((['--rescan-drawn-bonnet'] if physical else ['--full-bonnet']) if part == 'bonnet'
                       else ['--side', side] if side else [])
            exit_code = PartReplacer.run_child([sys.executable, '-u', '-m', 'repair_car.inspection.real_scan', '--part', part,
                                                '--controller', 'mesh-height', *options,
                                                '--result-manifest', str(manifest),
                                                *(['--inspection-position'] if physical else []), *ros_args],
                                               line, env=env)
            if exit_code != 0:
                raise RuntimeError(f'{name} 스캔 실패 (real_scan 종료 코드 {exit_code})')
            files = [Path(path).resolve() for path in json.loads(manifest.read_text())]
        archive = InspectionArchive.directory(vehicle, part, side).resolve()
        runs = [ScanFiles.from_plan(path) for path in files if path.name.endswith('_plan.json')]
        if not runs or any(run.directory != archive or not run.run.startswith(f'real_{name}_') for run in runs):
            raise RuntimeError(f'{name} 스캔 결과(result.json)를 찾을 수 없습니다 ({archive})')
        confirmed, candidates = self.scan_findings(runs)
        self.event('detect', f'{name}: 확정 {confirmed or "없음"}, 후보 {candidates or "없음"} '
                             f'({", ".join(run.run for run in runs)})', 'warn' if confirmed else 'info')
        findings += [dict(part_code=code, decision=kind) for kind in confirmed]   # dent / crack
        if candidates and not confirmed:
            findings.append(dict(part_code=code, decision='suspect'))
        return findings, bool(confirmed)

    # ---------------- virtual demo_final (run_virtual.sh): every panel on the lift turntable ----------------
    def process_demo(self, job):
        program, job_id = self.program, job['job_id']
        self.event('calibrate', f'job {job_id} (auto, 가상 demo_final): 보닛 → 왼쪽 문 → 오른쪽 문, 리프트 턴테이블')
        replacer, findings, parts = PartReplacer(program, self.event), [], []
        for panel, part, side, code, heading in DEMO_PANELS:
            self.retry('calibrate', f'{panel}: 탐침 파지 확인', self.hold_probe)
            self.retry('calibrate', f'{panel}: HOME 이동', program.home)
            self.retry('plan', f'{panel}: 턴테이블 회전·리프트 상승', lambda: self.place_demo(panel, heading))
            found, confirmed = self.retry('scan', f'{panel}: mesh-height 검출', lambda: self.scan_part(part, side, code))
            if found:
                self.db.findings(job_id, found)
            findings += found
            if not confirmed:
                self.event('detect', f'{panel}: 확정 손상 없음 — 교환 생략')
                continue
            self.check_stock(code)
            outcome = self.retry('replace', f'{code}: 교환', lambda: replacer.replace(code))
            self.event('inspect', f'{code}: flush {outcome["flush_mm"]} mm')
            self.db.replacement(job_id, code, **outcome)
            try:
                self.db.consume(code)
            except RuntimeError as error:
                self.event('stock', f'{code}: 재고 차감 실패 ({error})', 'warn')
            parts.append(code)
        self.retry('done', 'HOME 이동', program.home)
        self.event('done', f'replaced {parts} (검출 {len(findings)}건)')
        return ('done', f'교체 완료: {", ".join(parts)}') if parts else ('no_damage', '확정 손상 없음: 패널 검사 완료')

    def retry(self, stage, label, action):
        """Retry after returning HOME; the multi-panel virtual demo stops after three failed attempts."""
        attempt = 1
        while True:
            try:
                return action()
            except Exception as error:
                if self.program.robot.cancelled or getattr(error, 'no_home', False):
                    raise
                self.event(stage, f'{label} 실패 ({attempt}회째): {error} — 다시 시작', 'warn')
                if self.program.mode == 'virtual' and ScanScene.multi_panel() and attempt >= 3:
                    raise RuntimeError(f'{label}: 3회 실패로 작업 종료') from error
                attempt += 1
                try:
                    self.program.home()
                except Exception as home_error:
                    if self.program.robot.cancelled:
                        raise
                    self.program.log(f'HOME before the retry failed: {home_error}')
                time.sleep(1.)

    def place_demo(self, panel, heading):
        """Turntable heading and lift height to the real demo_final placement of `panel` (LiftCommand), then
        /fixture/select when the turn alone leaves the vehicle off its exact demo position."""
        from repair_car.geometry.real_vehicle import RealVehicle
        if not hasattr(self, 'stand'):
            from repair_car.lift.turnlift.virtual_stand import VirtualStand
            self.stand = VirtualStand()
            self.seat, self.size = self.stand.seat(RealVehicle().triangles), RealVehicle().size
        pose, base_z = ScanScene.inspection_pose(panel, vehicle_height_mm=self.size[2])
        table_z = load('real_scene')['vehicle']['table_z_mm']
        self.lift(f'heading {heading:g}')
        self.lift(f'lift {base_z - table_z - self.seat - self.stand.disc_mm:.3f}')
        if ScanScene.virtual_panel() != panel:
            ScanScene.prepare_virtual(self.program, panel)
        self.event('plan', f'{panel}: 배치 {pose} / 차량 z {base_z:.2f} mm')

    def lift(self, line):
        """One LiftCommand on the virtual stand (<namespace>/lift/command), waiting for the arm to settle at HOME."""
        return ScanScene.at_home(self.program, lambda: self.lift_once(line))

    def lift_once(self, line):
        robot = self.program.robot
        if not hasattr(self, 'lift_client'):
            self.lift_client = self.program.node.create_client(LiftCommand, 'lift/command')
        if not self.lift_client.wait_for_service(timeout_sec=10.):
            raise TimeoutError(f'{self.lift_client.srv_name} 서비스가 없습니다')
        future = self.lift_client.call_async(LiftCommand.Request(command=line))
        deadline = time.monotonic() + 15.
        while not future.done():
            robot.check_cancelled()
            if time.monotonic() > deadline:
                raise TimeoutError(f'lift/command 응답 시간 초과: {line}')
            robot.executor.spin_once(timeout_sec=.05)
        reply = future.result()
        if not reply.ok:
            raise RuntimeError(f'lift/command {line}: {reply.error}')
        self.event('plan', f'리프트 {line}: heading {reply.status.heading:.0f}°, 상판 {reply.status.stand_top_mm:.2f} mm, '
                           f'차량 z {reply.status.vehicle_z_mm:.2f} mm')

    def check_stock(self, code):
        _, missing = StockChecker(self.db).check([code])
        if missing:
            self.event('stock', f'{code}: 재고 없음 — 가상 시연이라 교환을 계속합니다', 'warn')

    def hold_probe(self):
        program = self.program
        if program.mode == 'real':
            return self.check_real_probe(pick=True)
        plan = read_session('probe_plan.json')
        if plan is None:
            raise RuntimeError('탐침 거치 계획(data/probe_plan.json)이 없습니다. --car 옵션으로 가상 세션을 시작하세요.')
        self.event('calibrate', '가상 탐침 파지 확인 및 준비')
        try:
            ProbePickup(program.node, program.robot, plan, program.config['motion']).run()
        except (RuntimeError, TimeoutError) as error:
            raise ReplaceAborted(f'가상 탐침 파지 확인 실패: {error}') from error


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--calibrate', action='store_true', help='calibrate at every job start')
    parser.add_argument('--job-id', type=int, help='Run one existing DB job and exit (web_control)')
    parser.add_argument('--front-placement-confirmed', action='store_true',
                        help='The operator has confirmed the front-facing bonnet fixture')
    options, _ = parser.parse_known_args()
    manager, code = RepairManager(options.calibrate, options.front_placement_confirmed), 1
    try:
        if options.job_id is not None:
            job = manager.db.job(options.job_id)
            if job['status'] != 'running':
                raise ValueError('The repair job is no longer running')
            manager.job = dict(job, job_id=options.job_id)
            code = manager.run(manager.job)
            return code
        manager.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if options.job_id is not None:
            # On the real robot destroy_node() once never returned after a finished job (2026-11-20, job 124, after a
            # burst of controller singularity alarms): web_control then showed the job running until a stop.
            # The job is recorded by now, so leave anyway.
            def leave():
                print(f'repair_manager: ROS shutdown did not finish within {CLOSE_TIMEOUT_S:.0f} s, forcing exit', flush=True)
                sys.stdout.flush()
                os._exit(code)
            watchdog = threading.Timer(CLOSE_TIMEOUT_S, leave)
            watchdog.daemon = True
            watchdog.start()
        manager.program.close()


if __name__ == '__main__':
    raise SystemExit(main())
