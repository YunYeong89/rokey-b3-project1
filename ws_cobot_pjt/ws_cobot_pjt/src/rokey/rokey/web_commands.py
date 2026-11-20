"""Commands exposed to the web UI. Only fixed modules and typed CLI options are accepted.
Spec: module (rokey.<module>), arguments (fixed CLI arguments), options, probe (needs the probe attached), scene (the
virtual session scene it needs: real, performance or model), virtual (simulation only), panel (started from its own
web panel, not the task list)."""
import math
import sys

from .part_scan import SCAN_CONTROLLERS
from .web_inspection import InspectionStep, PARTS, inspection_regions


class WebCommands:
    @staticmethod
    def number(label, flag, minimum, maximum, integer=False):
        return dict(label=label, flag=flag, type='integer' if integer else 'number',
                    min=minimum, max=maximum)

    @staticmethod
    def choice(label, flag, choices, empty=None, virtual=()):
        """empty: label of a first choice that passes no option; virtual: values for the simulation only (plan_only
        on the real robot)."""
        return dict(label=label, flag=flag, type='select', choices=list(choices), empty=empty, virtual=list(virtual))

    @staticmethod
    def toggle(label, flag):
        return dict(label=label, flag=flag, type='boolean')

    def __init__(self):
        n, c, b = self.number, self.choice, self.toggle
        speed = n('이송 속도 [mm/s]', '--speed', .1, 10)
        pitch = n('행 간격 [mm]', '--pitch', .1, 10)
        plan_only = b('계획만 확인 (로봇 이동 없음)', '--plan-only')
        self.commands = {
            'robot_demo': dict(label='연결 확인 데모 (HOME → 그리퍼 닫기·열기)', module='robot_demo', options={
                'cycles': n('그리퍼 반복 횟수', '--cycles', 1, 5, True),
                'no_gripper': b('HOME만 확인 (그리퍼 생략)', '--no-gripper')}),
            'home': dict(label='HOME 복귀', module='web_home', options={}),
            # The web's teleop popup (robot_teleop.js) starts it; its buttons publish /robot/teleop.
            'teleop': dict(label='Teleop (웹 수동 조작)', module='teleop', arguments=['--web'], panel='teleop',
                           options={}),
            'calibration': dict(label='접촉·힘 보정', module='calibration', probe=True, options={
                'skip_force': b('기존 힘 모델 유지', '--skip-force')}),
            'real_scan': dict(label='부품 검사', module='real_scan', scene='real', panel='inspection', options={
                'part': c('부품', '--part', ('bonnet', 'bumper', 'body', 'door')),
                'side': c('도어 면', '--side', ('left', 'right'), empty='양쪽 (왼쪽 → 오른쪽)'),
                'controller': c('제어기', '--controller', SCAN_CONTROLLERS, virtual=('mesh-normal',)),
                'plan_only': plan_only, 'replan': b('계획 캐시 다시 만들기 (mesh 제어기)', '--replan'),
                'pitch': pitch, 'speed': speed}),
        }

    def catalog(self):
        return [dict(id=key, **{k: v for k, v in command.items() if k not in ('module', 'arguments')})
                for key, command in self.commands.items()]

    def nodes(self):
        """Node names of the robot programs (RobotProgram(name)), to see ones started from a terminal."""
        return {spec['module'] for spec in self.commands.values()} | {
            'choose_car', 'scan_surface', 'defect_scan', 'pick_probe', 'release_probe', 'pid_surface', 'repair_manager'}

    def inspection(self, requests, plan_only, mode):
        if not isinstance(requests, list) or not requests or type(plan_only) is not bool:
            raise ValueError('검사할 부품을 하나 이상 지정하세요')
        steps, seen = [], set()
        for item in requests:
            if not isinstance(item, dict) or set(item) != {'part', 'regions'}:
                raise ValueError('부품과 탐색 영역이 필요합니다')
            part = item['part']
            if not isinstance(part, str) or part not in PARTS or part in seen:
                raise ValueError('보닛, 왼쪽 도어, 오른쪽 도어를 한 번씩 선택할 수 있습니다')
            seen.add(part)
            physical_part, side = PARTS[part]
            base = self.build('real_scan', dict(part=physical_part, side=side, controller='mesh-height',
                                               plan_only=plan_only), mode)
            regions = inspection_regions(item['regions'])
            for number, box in enumerate(regions or [None], 1 if regions else 0):
                arguments = base + (['--inspection-position'] if mode == 'real' else [])
                if box is not None:
                    arguments += ['--roi-local', *map(str, box), '--inspection-region', str(number)]
                steps.append(InspectionStep(part, number, arguments))
        return steps

    def build(self, name, parameters, mode):
        if name not in self.commands:
            raise ValueError('지원하지 않는 기능입니다')
        spec = self.commands[name]
        if spec.get('virtual') and mode != 'virtual':
            raise ValueError('시뮬레이션 전용 기능입니다')
        if not isinstance(parameters, dict) or parameters.keys() - spec['options'].keys():
            raise ValueError('지원하지 않는 실행 옵션입니다')
        arguments = [sys.executable, '-u', '-m', f'rokey.{spec["module"]}', *spec.get('arguments', [])]
        for key, value in parameters.items():
            field = spec['options'][key]
            kind = field['type']
            if kind == 'boolean':
                if not isinstance(value, bool):
                    raise ValueError(f'{key}: true/false 필요')
                if value:
                    arguments.append(field['flag'])
                continue
            if kind == 'select':
                if value == '' and field['empty']:
                    continue
                if value not in field['choices']:
                    raise ValueError(f'{key}: 잘못된 선택 값')
                if value in field['virtual'] and mode != 'virtual' and parameters.get('plan_only') is not True:
                    raise ValueError(f'{key}={value}: 실제 로봇에서는 계획 확인만 가능합니다')
            elif (isinstance(value, bool) or not isinstance(value, (int, float))
                  or not math.isfinite(value) or not field['min'] <= value <= field['max']
                  or (kind == 'integer' and int(value) != value)):
                raise ValueError(f'{key}: {field["min"]}~{field["max"]} 범위의 {kind} 필요')
            arguments.extend([field['flag'], str(int(value) if kind == 'integer' else value)])
        if name == 'real_scan' and 'part' not in parameters:
            arguments.extend(['--part', 'bonnet'])
        return arguments
