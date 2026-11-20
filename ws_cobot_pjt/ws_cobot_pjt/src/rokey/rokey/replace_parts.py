"""Part replacement. The interface the process uses is final:
  PartReplacer(program).replace(part) -> {'success', 'retries', 'peak_force', 'flush_mm'}

Parts in TAUGHT run the pendant-taught DRL port (rokey.part_replace.<module>) as a child process, virtual or real:
the bonnet is disassembled then assembled at its taught positions (no vehicle identification needed). The child
gets the same SIGINT as this process on a web stop (one process group) and stops the robot and releases force and
compliance itself; it is waited for, not killed. A failure there raises ReplaceAborted: the robot may hold a part
next to the vehicle, so the process must not move it home (recover with the pendant).

Other parts are still a skeleton (tool change, grasp shapes, tray and scrap positions, force-controlled mounting,
flush re-inspection): in the simulation every step is only logged and succeeds, the real robot refuses.
  ros2 run rokey replace_parts --parts bonnet
"""
import argparse
import re
import subprocess
import sys
import threading

if __package__:
    from .robot_namespace import program_args
else:
    from robot_namespace import program_args

if __package__:
    from .program import RobotProgram
else:
    from program import RobotProgram

# part -> (module, [(mode argument, label), ...]) run in order
TAUGHT = {'bonnet': ('rokey.part_replace.bonnet_replace', [('disassembly', '분해'), ('assembly', '조립')])}
STEP_LINE = re.compile(r'\[(분해|조립) (\d+)/(\d+)\] (.*?)(?:\s+\(현재 z=[^)]*\))?\s*$')  # bonnet_replace begin()
STOP_LINE = re.compile(r'(\[(?:분해|조립)\] 중단됨.*|에러로 중단됨: .*|Ctrl\+C 로 중단됨.*)$')
CHILD_STOP_WAIT_S = 12.0  # after a stop: time the child gets to stop and release (web_control kills at 15 s)


class ReplaceAborted(RuntimeError):
    """A taught replacement stopped part-way: the process records the failure but does not move the robot home."""
    no_home = True


class PartReplacer:
    steps = ['tool change: probe -> part gripper', 'remove the damaged part', 'drop it at the scrap position',
             'pick a new part from the tray', 'mount it under TOOL Z force control', 'tool change: part gripper -> probe',
             'flush re-inspection along the part outline']

    def __init__(self, program, event=None):
        """event(stage, message): progress for the repair DB (repair_manager); the log only when not given."""
        self.program = program
        self.event = event or (lambda stage, message: program.log(f'[{stage}] {message}'))

    def replace(self, part):
        if part in TAUGHT:
            return self.replace_taught(part)
        if self.program.mode != 'virtual':
            raise NotImplementedError('실기 부품 교체 동작은 아직 구현되지 않았습니다 (replace_parts 뼈대)')
        for index, step in enumerate(self.steps):
            self.program.log(f'{part} {index + 1}/{len(self.steps)}: {step} (simulated, no motion)')
        self.program.publish(type='replaced', part=part)
        return {'success': True, 'retries': 0, 'peak_force': None, 'flush_mm': None}

    def replace_taught(self, part):
        module, phases = TAUGHT[part]
        virtual = ['--virtual'] if self.program.mode == 'virtual' else []
        ros_args = ['--ros-args', '-r', f'__ns:={self.program.node.get_namespace()}']
        for phase, label in phases:
            self.event('replace', f'{part} {label} 시작 (티칭 좌표)')
            stopped = []

            def line(text):
                """The child's step lines ([분해 3/9] ...) also go to the repair DB log."""
                step = STEP_LINE.search(text)
                if step:
                    self.event('replace', f'{part} {step[1]} {step[2]}/{step[3]}: {step[4]}')
                elif STOP_LINE.search(text):
                    stopped.append(STOP_LINE.search(text)[1])

            code = self.run_child([sys.executable, '-u', '-m', module, phase, *virtual, *ros_args], line)
            if code != 0:
                reason = f' — {stopped[-1]}' if stopped else ''
                raise ReplaceAborted(f'{part} {label} 중단 (종료 코드 {code}{reason}): 로봇을 펜던트로 안전한 곳까지 옮기세요')
            self.event('replace', f'{part} {label} 완료')
        self.program.publish(type='replaced', part=part)
        return {'success': True, 'retries': 0, 'peak_force': None, 'flush_mm': None}

    @staticmethod
    def run_child(arguments, on_line=None):
        """subprocess.run would SIGKILL the child 0.25 s after a Ctrl+C, cutting its force/compliance release.
        on_line(text): each output line (passed on to this process's output too), read on a thread that keeps
        draining the pipe while the child stops and releases after a stop."""
        if on_line is None:
            child, reader = subprocess.Popen(arguments), None
        else:
            child = subprocess.Popen(arguments, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                                     bufsize=1, errors='replace')

            def read():
                for text in child.stdout:
                    sys.stdout.write(text)
                    sys.stdout.flush()
                    try:
                        on_line(text.rstrip('\n'))
                    except Exception as error:   # a DB hiccup must not stop the motion's output
                        print(f'replace_parts: 로그 기록 실패: {error}', flush=True)

            reader = threading.Thread(target=read, daemon=True)
            reader.start()
        try:
            return child.wait()
        except KeyboardInterrupt:
            try:
                child.wait(timeout=CHILD_STOP_WAIT_S)
            except subprocess.TimeoutExpired:
                child.kill()
            raise
        finally:
            if reader is not None:
                reader.join(timeout=2.0)


def run(parts):
    program = RobotProgram('replace_parts')
    try:
        replacer = PartReplacer(program)
        for part in parts:
            program.log(f'{part}: {replacer.replace(part)}')
    finally:
        program.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--parts', nargs='+', required=True, help='part codes, e.g. bonnet mirror_r')
    run(parser.parse_args(program_args()).parts)


if __name__ == '__main__':
    main()
