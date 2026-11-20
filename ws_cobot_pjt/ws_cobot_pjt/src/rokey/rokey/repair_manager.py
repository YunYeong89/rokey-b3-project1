"""Repair process node (INTERFACE.md section 5). Waits for /repair/command and runs one job at a time,
recording every stage in the repair DB:

  calibrate  probe pickup (simulation, when not held), calibration when there is no result for this mode
             (or --calibrate), the vehicle at its fixed pose (choose_car.fixed: no identification touches). A manual
             job of taught parts only (replace_parts.TAUGHT: the bonnet) skips calibration and the vehicle pose:
             those parts are replaced at pendant-taught positions.
  scan       auto mode. The real vehicle (real robot, or a virtual session of the real assembly): real_scan
             (--controller mesh-height, as the web part inspection) per REAL_SCAN_PARTS -> detect: its result.json
             damage summary, a confirmed dent / crack ROI -> finding and replace, candidates only -> suspect.
             The scaled model vehicles (virtual sedan/suv/sports): scan_surface over the replaceable parts.
             No damage -> no_damage
  stock      check_db for the parts to replace (auto: dent/missing, manual: the requested parts) -> on_hold
  replace    replace_parts per part -> inspect -> replacement record, consume on success -> done / failed

A stop command stops the robot at once and records nothing more (the web records `stopped`); an error is
recorded, the job fails and the robot goes home (not after a taught replacement stopped part-way: it may hold
a part next to the vehicle).
  ros2 run rokey repair_manager [--calibrate]          (or ros2 launch rokey repair.launch.py)
"""
import argparse
import json
import os
import sys
import threading

from std_msgs.msg import String

if __package__:
    from .calibration import calibrate
    from .check_db import StockChecker
    from .choose_car import fixed
    from .configs import output_dir, read_session
    from .robot_namespace import robot_topic
    from .force_data import CalibrationRun
    from .repair_db import RepairDB
    from .replace_parts import TAUGHT, PartReplacer
    from .scan_surface import scan_parts
else:
    from calibration import calibrate
    from check_db import StockChecker
    from choose_car import fixed
    from configs import output_dir, read_session
    from robot_namespace import robot_topic
    from force_data import CalibrationRun
    from repair_db import RepairDB
    from replace_parts import TAUGHT, PartReplacer
    from scan_surface import scan_parts


REAL_SCAN_PARTS = ('bonnet',)  # real vehicle: parts scanned in auto mode (only the bonnet can be replaced so far)
CLOSE_TIMEOUT_S = 5.0  # --job-id: exit anyway when the ROS shutdown hangs (the job is already recorded)


class Stopped(Exception):
    """The job ended early with a final status (no_damage, on_hold)."""

    def __init__(self, status, summary):
        super().__init__(summary)
        self.status = status


class RepairManager:
    def __init__(self, always_calibrate):
        self.program = CalibrationRun('repair_manager')
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
                    if not getattr(error, 'no_home', False):
                        program.home()
            return 1
        finally:
            program.robot.cancelled = False

    def process(self, job):
        program, job_id = self.program, job['job_id']
        manual = job.get('mode') == 'manual'
        parts = self.db.requested_parts(job_id) if manual else None
        real = self.real_vehicle()
        if manual and parts and all(part in TAUGHT for part in parts):
            self.event('calibrate', f'job {job_id} (manual {parts}): 티칭 좌표 교체라 보정·차량 인식 생략, 탐침 확인')
            self.hold_probe()   # the bonnet disassembly starts by putting the held probe on its holder
        elif not manual and real:
            self.event('calibrate', f'job {job_id} (auto, 실차): 실측 배치(real_scene.yaml)로 스캔, 보정·차량 인식 생략')
            self.hold_probe()
        else:
            self.event('calibrate', f'job {job_id} ({job.get("mode", "auto")}): probe, calibration, fixed vehicle pose')
            self.hold_probe()
            result = read_session('calibration_result.json')
            if self.always_calibrate or not result or result['mode'] != program.mode:
                calibrate(program)
            vehicle = fixed(program)
            self.event('calibrate', f'vehicle {vehicle["type"]} at {[round(v, 1) for v in vehicle["pose_base_mm_deg"]]} '
                                    f'(고정 위치, {vehicle["source"]})')
        if not manual and real:
            findings, parts = self.scan_real()
            if findings:
                self.db.findings(job_id, findings)
            else:
                raise Stopped('no_damage', '손상 없음')
            if not parts:
                raise Stopped('done', '교체 대상 없음 (의심 판정만 있음)')
        elif not manual:
            self.event('scan', f'parts {program.config["inspection"]["replaceable"]}')
            scan = scan_parts(program, program.config['inspection']['replaceable'])
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
            raise Stopped('on_hold', f'재고 없음: {", ".join(missing)}')
        replacer, failed = PartReplacer(program, self.event), []
        for part in parts:
            self.event('replace', part)
            outcome = replacer.replace(part)
            self.event('inspect', f'{part}: flush {outcome["flush_mm"]} mm')
            if outcome['flush_mm'] is not None:
                program.publish(type='flush', mm=outcome['flush_mm'])
            self.db.replacement(job_id, part, **outcome)
            if outcome['success']:
                self.db.consume(part)
            else:
                failed.append(part)
        program.home()
        self.event('done', f'replaced {[part for part in parts if part not in failed]}')
        return ('failed', f'교체 실패: {", ".join(failed)}') if failed else ('done', f'교체 완료: {", ".join(parts)}')

    def real_vehicle(self):
        """The team's real vehicle: the real robot, or a virtual session of the real assembly (--car real / demo)."""
        truth = read_session('scene_truth.json') if self.program.mode == 'virtual' else None
        return self.program.mode == 'real' or (truth or {}).get('model') == 'real'

    def scan_real(self):
        """real_scan per REAL_SCAN_PARTS as a child process (stopped with this process group), then its damage
        summary (images/real_<part>_<time>/result.json). Returns (findings, parts to replace)."""
        program, images = self.program, output_dir('images')
        ros_args = ['--ros-args', '-r', f'__ns:={program.node.get_namespace()}']
        findings, replace = [], []
        for part in REAL_SCAN_PARTS:
            self.event('scan', f'{part}: 실차 스캔 (real_scan, mesh-height) 시작')
            before = set(images.glob(f'real_{part}_*'))
            code = PartReplacer.run_child([sys.executable, '-u', '-m', 'rokey.real_scan', '--part', part,
                                           '--controller', 'mesh-height',
                                           *(['--inspection-position'] if program.mode == 'real' else []), *ros_args])
            if code != 0:
                raise RuntimeError(f'{part} 스캔 실패 (real_scan 종료 코드 {code})')
            runs = sorted(path for path in set(images.glob(f'real_{part}_*')) - before if (path / 'result.json').exists())
            if not runs:
                raise RuntimeError(f'{part} 스캔 결과(result.json)를 찾을 수 없습니다 ({images})')
            damage = json.loads((runs[-1] / 'result.json').read_text()).get('damage') or {}
            regions = damage.get('regions') or {}
            confirmed = {kind: info['confirmed'] for kind, info in regions.items() if info.get('confirmed')}
            candidates = {kind: info['candidates'] for kind, info in regions.items() if info.get('candidates')}
            percent = damage.get('damage_percent')
            self.event('detect', f'{part}: 손상 {"-" if percent is None else f"{percent:.1f}%"}, 확정 {confirmed or "없음"}, '
                                 f'후보 {candidates or "없음"} ({runs[-1].name})', 'warn' if confirmed else 'info')
            findings += [dict(part_code=part, decision=kind) for kind in confirmed]   # dent / crack
            if confirmed:
                replace.append(part)
            elif candidates:
                findings.append(dict(part_code=part, decision='suspect'))
        return findings, replace

    def hold_probe(self):
        """In the simulation, pick the probe from its rack unless the probe TCP is already active."""
        program, probe = self.program, self.program.config['probe']
        if program.mode != 'virtual' or program.robot.get_tcp() == probe['tcp_name']:
            return
        if __package__:
            from .pick_probe import ProbePickup
        else:
            from pick_probe import ProbePickup
        plan = read_session('probe_plan.json')
        if plan is None:
            raise RuntimeError('session/probe_plan.json 없음: ./run_virtual.sh --car sedan (suv, sports, random) 으로 세션을 시작하세요')
        ProbePickup(program.node, program.robot, plan, program.config['motion']).run()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--calibrate', action='store_true', help='calibrate at every job start')
    parser.add_argument('--job-id', type=int, help='Run one existing DB job and exit (web_control)')
    options, _ = parser.parse_known_args()
    manager, code = RepairManager(options.calibrate), 1
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
                print(f'repair_manager: ROS 종료가 {CLOSE_TIMEOUT_S:.0f}초 안에 끝나지 않아 강제 종료', flush=True)
                sys.stdout.flush()
                os._exit(code)
            watchdog = threading.Timer(CLOSE_TIMEOUT_S, leave)
            watchdog.daemon = True
            watchdog.start()
        manager.program.close()


if __name__ == '__main__':
    raise SystemExit(main())
