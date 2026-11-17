"""Commands exposed to the web UI. Only fixed modules and typed CLI options are accepted."""
import math
import sys


class WebCommands:
    @staticmethod
    def number(label, flag, minimum, maximum, integer=False):
        return dict(label=label, flag=flag, type='integer' if integer else 'number',
                    min=minimum, max=maximum)

    @staticmethod
    def choice(label, flag, choices):
        return dict(label=label, flag=flag, type='select', choices=list(choices))

    @staticmethod
    def toggle(label, flag):
        return dict(label=label, flag=flag, type='boolean')

    def __init__(self):
        n, c, b = self.number, self.choice, self.toggle
        rows = n('측정 행 수 (비우면 전체)', '--rows', 1, 1000, True)
        speed = n('이송 속도 [mm/s]', '--speed', .1, 10)
        pitch = n('행 간격 [mm]', '--pitch', .1, 10)
        self.commands = {
            'home': dict(label='HOME 복귀', module='web_home', options={}),
            'pick_probe': dict(label='탐침 파지', module='pick_probe', options={}),
            'calibration': dict(label='접촉·힘 보정', module='calibration', probe=True, options={
                'skip_force': b('기존 힘 모델 유지', '--skip-force')}),
            'choose_car': dict(label='차량 식별', module='choose_car', probe=True, model=True, options={}),
            'scan_surface': dict(label='차량 부품 스캔·판정', module='scan_surface', probe=True, model=True, options={}),
            'defect_scan': dict(label='블록 결함 스캔', module='defect_scan', probe=True, options={
                'rows': rows, 'pitch': pitch, 'speed': speed}),
            'real_scan': dict(label='실차 모델 부품 스캔 (가상 장면)', module='real_scan', probe=True, real_scene=True, options={
                'part': c('부품', '--part', ('bonnet', 'bumper', 'body', 'door')),
                'plan_only': b('계획만 확인 (로봇 이동 없음)', '--plan-only'), 'pitch': pitch, 'speed': speed}),
        }

    def catalog(self):
        return [dict(id=key, **{k: v for k, v in command.items() if k != 'module'})
                for key, command in self.commands.items()]

    def build(self, name, parameters):
        if name not in self.commands:
            raise ValueError('지원하지 않는 기능입니다')
        spec = self.commands[name]
        if not isinstance(parameters, dict) or parameters.keys() - spec['options'].keys():
            raise ValueError('지원하지 않는 실행 옵션입니다')
        arguments = [sys.executable, '-u', '-m', f'rokey.{spec["module"]}']
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
                if value not in field['choices']:
                    raise ValueError(f'{key}: 잘못된 선택 값')
            elif (isinstance(value, bool) or not isinstance(value, (int, float))
                  or not math.isfinite(value) or not field['min'] <= value <= field['max']
                  or (kind == 'integer' and int(value) != value)):
                raise ValueError(f'{key}: {field["min"]}~{field["max"]} 범위의 {kind} 필요')
            arguments.extend([field['flag'], str(int(value) if kind == 'integer' else value)])
        if name == 'real_scan' and 'part' not in parameters:
            arguments.extend(['--part', 'bonnet'])
        return arguments
