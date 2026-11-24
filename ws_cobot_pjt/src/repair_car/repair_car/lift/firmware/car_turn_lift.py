#!/usr/bin/env python3
# 4륜 TT 모터 자동차 제자리 회전(0/90/180/270°) + MG996R 리프트 — Ubuntu 22.04 / Raspberry Pi
#
#   5 : 0°(처음 방향)   6 : 90°   7 : 180°   8 : 270°
#   3 : 왼쪽 90°        4 : 오른쪽 90°
#   1 : 리프트 올림(+62 mm)   2 : 리프트 내림
#   c : 회전 보정 (처음 한 번)
#   q : 0°로 돌아오고 리프트 내린 뒤 종료
#
# 로봇 연동(lift/README.md): sudo python3 car_turn_lift.py --serve 5005
#   TCP 한 줄 명령: heading 90 | turn -90 | lift up | lift down | lift 100(mm, lift_table.json)
#                  station top | station front | drive -180(mm, + 앞) | status | quit
#   station: 0°로 돌린 뒤 직진으로 위 측정 자리(시작 위치) ↔ 정면 측정 자리(로봇에서 STATION_MM 더 먼 곳) 이동
#   응답은 JSON 한 줄. 프로세스가 살아 있는 동안 리프트 서보가 하중을 계속 버팁니다(PWM 유지).
#
# 회전 속도: 90° 를 3초에 (TIME_90). 180° 는 6초.
# 엔코더가 없는 TT 모터라 "시간"으로 각도를 맞추고, 그 시간에 정확히 90° 가 되도록
# 모터 출력(TURN_DUTY)을 'c' 보정으로 맞춥니다.
#
# 라즈베리파이: sudo apt install python3-lgpio   →  sudo python3 car_turn_lift.py
# Windows    : python car_turn_lift.py   (GPIO 가 없으므로 모의 실행 — 동작을 화면에만 표시)

import argparse
import json
import os
import socket
import sys
import time

IS_WINDOWS = os.name == "nt"
if IS_WINDOWS:
    import msvcrt
    try:
        sys.stdout.reconfigure(errors="replace")      # 콘솔 글꼴에 없는 기호는 ? 로
    except Exception:
        pass
else:
    import termios
    import tty

try:
    import lgpio
    SIM = False
except ImportError:
    SIM = True

    class _FakeLgpio:
        """GPIO 가 없는 PC 에서 쓰는 가짜 lgpio (아무 핀도 건드리지 않음)"""
        error = OSError

        def gpiochip_open(self, chip): return 0
        def gpio_claim_output(self, h, pin, level=0): pass
        def gpio_write(self, h, pin, level): pass
        def tx_pwm(self, h, pin, freq, duty): pass
        def tx_servo(self, h, pin, width, freq=50): pass
        def gpiochip_close(self, h): pass

    lgpio = _FakeLgpio()


# ---------------- 키보드 (Windows / 리눅스 공통) ----------------
class Keys:
    def __enter__(self):
        if not IS_WINDOWS:
            self.fd = sys.stdin.fileno()
            self.old = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)
        return self

    def __exit__(self, *a):
        self.normal()

    def get(self):
        if IS_WINDOWS:
            ch = msvcrt.getwch()
            if ch in ("\x00", "\xe0"):          # 방향키 등 특수키는 무시
                msvcrt.getwch()
                return ""
            if ch == "\x03":
                raise KeyboardInterrupt
            return ch
        return sys.stdin.read(1)

    def flush(self):
        """움직이는 동안 눌린 키 버리기"""
        if IS_WINDOWS:
            while msvcrt.kbhit():
                msvcrt.getwch()
        else:
            termios.tcflush(self.fd, termios.TCIFLUSH)

    def normal(self):
        if not IS_WINDOWS:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

    def raw(self):
        if not IS_WINDOWS:
            tty.setcbreak(self.fd)

# ================= 모터 드라이버 핀 (다두이노 RaspberryPi AI CAR 보드, TB6612, BCM 번호) =================
DRIVER = "EN_IN"          # TB6612: PWM + IN1/IN2 방식
LEFT = {"en": 18, "in1": 22, "in2": 27}    # A채널: PWMA=GPIO18, AIN1=GPIO22, AIN2=GPIO27
RIGHT = {"en": 25, "in1": 23, "in2": 24}   # B채널: PWMB=GPIO25, BIN1=GPIO23, BIN2=GPIO24 (책 표 기준)
#   ※ 책의 회로도에는 PWMB=GPIO23, BIN1=GPIO25 로 반대로 적혀 있습니다.
#     오른쪽 바퀴가 한 방향으로만 돌거나 속도 조절이 안 되면 아래 줄로 바꾸세요.
# RIGHT = {"en": 23, "in1": 25, "in2": 24}
#   TWO_PIN 방식이면 {"a": 핀, "b": 핀} 형태로 적으세요.
STBY = None               # 보드에서 STBY 가 VCC 에 연결돼 있어 따로 제어하지 않음
INVERT_LEFT = False       # 'c' 보정 때 왼쪽 바퀴가 반대로 돌면 True
INVERT_RIGHT = False
BRAKE_HOLD = True         # 회전 후 TB6612 단락 제동(IN1 = IN2 = H): 탐침이 밀어도 바퀴가 굴러가지 않게
DRIVE_DUTY = 30           # 직진 출력 %
DRIVE_MM_S = 114.0        # DRIVE_DUTY 에서의 직진 속도 (MuJoCo 추정, 실측으로 교체). 엔코더가 없어 시간으로 거리 환산
STATION_MM = 180.0        # device.yaml scan.directions front.centre_mm - top.centre_mm (heading 0 에서 뒤로)

# ================= 회전 =================
TIME_90 = 3.0             # 90° 도는 데 걸리는 시간 (초). 출력 100% 로도 모자라면 'c' 보정이 이 시간을 늘림
TURN_DUTY = 35            # 시작 출력 %. 'c' 보정으로 자동 조정되어 저장됨
DEAD_DUTY = 10            # 이 출력 이하로는 바퀴가 안 돈다고 보는 값 (보정 계산용)
KICK_DUTY, KICK_S = 70, 0.08   # 출발할 때 잠깐 세게 (느린 속도에서 멈춤 방지)
RAMP_S = 0.4              # 서서히 출발·정지하는 시간
PWM_FREQ = 100            # 낮은 주파수가 저속에서 힘이 더 좋음

# ================= 리프트 =================
LIFT_PIN = 12             # GPIO18 은 모터(PWMA)가 쓰므로 리프트 서보는 GPIO12 (물리 핀 32)
# 90×60 리프트(hardware/lift_assembly.stl): 서보 172° = 내림(크랭크 수평), 119° = 올림(크랭크 53°, +12.3 mm)
# 크랭크 54.6° 에서 슬라이더가 슬롯 끝에 닿으므로 올림은 그보다 1.6° 앞에서 멈춤 (config/device.yaml linkage)
# 처음 '1' 을 눌렀을 때 크랭크가 아래(바닥 쪽)로 돌면 True 로 바꾸세요 → 8° 내림 / 61° 올림
LIFT_REVERSED = False
DOWN_ANGLE, UP_ANGLE = (8, 61) if LIFT_REVERSED else (172, 119)
LIFT_SPEED = 40           # 도/초 (53° 를 약 1.3초에)
LIFT_STATE = os.path.expanduser("~/.lift_state.json")   # 마지막 리프트 각도 기억 (켤 때 갑자기 튀지 않게)
LIFT_TABLE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "lift_table.json")  # python -m repair_car.lift.turnlift table

CALIB_FILE = os.path.expanduser("~/.car_turn_calib.json")
HEADINGS = {"5": 0, "6": 90, "7": 180, "8": 270}


def lift_pulse(a):
    return int(500 + max(0, min(180, a)) * 2000 / 180)


def lift_table():
    """(서보 각도 목록, 상판 높이 mm 목록) — 높이가 증가하는 순서. 표가 없으면 None"""
    try:
        with open(LIFT_TABLE) as f:
            t = json.load(f)
    except OSError:
        return None
    angles = [180 - a for a in t["servo_deg"]] if LIFT_REVERSED else t["servo_deg"]
    return angles, t["top_mm"]


def interp(x, xs, ys):
    """단조 xs(증가·감소 무관) 구간 선형 보간, 범위 밖은 끝값"""
    if xs[0] > xs[-1]:
        xs, ys = xs[::-1], ys[::-1]
    if x <= xs[0]:
        return ys[0]
    for (x0, y0), (x1, y1) in zip(zip(xs, ys), zip(xs[1:], ys[1:])):
        if x <= x1:
            return y0 + (y1 - y0) * (x - x0) / (x1 - x0)
    return ys[-1]


class Car:
    def __init__(self):
        try:
            self.h = lgpio.gpiochip_open(0)
            lgpio.gpio_claim_output(self.h, LIFT_PIN)
        except lgpio.error:
            self.h = lgpio.gpiochip_open(4)
            lgpio.gpio_claim_output(self.h, LIFT_PIN)
        for side in (LEFT, RIGHT):
            for p in side.values():
                lgpio.gpio_claim_output(self.h, p, 0)
        if STBY is not None:
            lgpio.gpio_claim_output(self.h, STBY, 1)
        self.lift = self._load_lift()
        lgpio.tx_servo(self.h, LIFT_PIN, lift_pulse(self.lift), 50)     # 마지막 위치에서 시작
        if abs(self.lift - DOWN_ANGLE) > 1:
            print(f"It was switched off with the lift raised ({self.lift:.0f}°). Press 2 to lower it.")
        self.heading = 0.0
        self.station = "top"
        self.duty, self.time_90 = float(TURN_DUTY), TIME_90
        try:
            with open(CALIB_FILE) as f:
                calib = json.load(f)
            self.duty, self.time_90 = float(calib["duty"]), float(calib.get("time_90", TIME_90))
            print(f"Calibration loaded: turn output {self.duty:.0f}%  (90° / {self.time_90:g} s)")
        except Exception:
            print(f"Turn output {self.duty:.0f}% (uncalibrated). Press 'c' to calibrate 90° to {TIME_90:g} s")

    # ---------------- 모터 ----------------
    def _side(self, pins, duty, invert):
        if invert:
            duty = -duty
        if DRIVER == "EN_IN":
            lgpio.gpio_write(self.h, pins["in1"], 1 if duty > 0 else 0)
            lgpio.gpio_write(self.h, pins["in2"], 1 if duty < 0 else 0)
            lgpio.tx_pwm(self.h, pins["en"], PWM_FREQ, abs(duty))
        else:
            if duty >= 0:
                lgpio.gpio_write(self.h, pins["b"], 0)
                lgpio.tx_pwm(self.h, pins["a"], PWM_FREQ, abs(duty))
            else:
                lgpio.tx_pwm(self.h, pins["a"], PWM_FREQ, 0)
                lgpio.gpio_write(self.h, pins["a"], 0)
                lgpio.tx_pwm(self.h, pins["b"], PWM_FREQ, abs(duty))

    def _spin(self, direction, duty):
        """direction +1 = 오른쪽(시계), -1 = 왼쪽"""
        self._side(LEFT, direction * duty, INVERT_LEFT)
        self._side(RIGHT, -direction * duty, INVERT_RIGHT)

    def stop(self):
        self._spin(0, 0)

    def brake(self):
        """TB6612 단락 제동: 기어박스가 차를 붙잡아 탐침 힘에 밀리지 않게 함"""
        for pins in (LEFT, RIGHT):
            if DRIVER == "EN_IN":
                lgpio.tx_pwm(self.h, pins["en"], PWM_FREQ, 0)
                lgpio.gpio_write(self.h, pins["in1"], 1)
                lgpio.gpio_write(self.h, pins["in2"], 1)
            else:
                lgpio.tx_pwm(self.h, pins["a"], PWM_FREQ, 0)
                lgpio.gpio_write(self.h, pins["a"], 1)
                lgpio.gpio_write(self.h, pins["b"], 1)

    def spin_for(self, direction, seconds):
        """출발 킥 → 서서히 가속 → 일정 속도 → 서서히 감속. 총 seconds 초.
        보정도 같은 모양으로 하므로 램프·킥 영향은 보정값에 포함됨."""
        d = self.duty
        if SIM:
            print(f"  [sim] wheels turn {'right' if direction > 0 else 'left'}: {seconds:.1f} s, output {d:.0f}%")
        self._spin(direction, KICK_DUTY)
        time.sleep(KICK_S)
        r = min(RAMP_S, max(0.0, (seconds - KICK_S) / 2))
        steps = max(1, int(r / 0.02))
        t0 = time.monotonic()
        for i in range(1, steps + 1):                       # 가속 (킥 후 바로 d 의 50% 부터)
            self._spin(direction, d * (0.5 + 0.5 * i / steps))
            time.sleep(r / steps)
        flat = seconds - KICK_S - 2 * r
        if flat > 0:
            time.sleep(flat)
        for i in range(steps, 0, -1):                       # 감속
            self._spin(direction, d * (0.5 + 0.5 * (i - 1) / steps))
            time.sleep(r / steps)
        self.brake() if BRAKE_HOLD else self.stop()

    # ---------------- 회전 ----------------
    def turn_by(self, deg):
        if abs(deg) < 1:
            return
        direction = 1 if deg > 0 else -1
        self.spin_for(direction, abs(deg) / 90.0 * self.time_90)
        self.heading = (self.heading + deg) % 360

    def drive_by(self, mm):
        """직진 (+ 앞, - 뒤). 시간 환산이라 오차가 있으니 스테이션마다 멈춤 블록·기준면 터치로 위치를 맞춤"""
        if abs(mm) < 1:
            return
        s = 1 if mm > 0 else -1
        if SIM:
            print(f"  [sim] straight {mm:+.0f} mm: {abs(mm) / DRIVE_MM_S:.1f} s, output {DRIVE_DUTY}%")
        self._side(LEFT, s * DRIVE_DUTY, INVERT_LEFT)
        self._side(RIGHT, s * DRIVE_DUTY, INVERT_RIGHT)
        time.sleep(abs(mm) / DRIVE_MM_S)
        self.brake() if BRAKE_HOLD else self.stop()

    def station_to(self, name):
        """top = 시작 자리, front = 로봇에서 STATION_MM 먼 자리. heading 0 (차 앞이 로봇 쪽)에서 앞뒤로만 이동"""
        if name == self.station:
            return
        self.turn_to(0)
        self.drive_by(-STATION_MM if name == "front" else STATION_MM)
        self.station = name

    def turn_to(self, target):
        d = (target - self.heading + 180) % 360 - 180      # 가까운 쪽으로
        if abs(abs(d) - 180) < 1:
            d = 180                                        # 정확히 반대면 오른쪽으로
        if abs(d) < 1:
            print(f"Already at {target}°")
            return
        print(f"{'right' if d > 0 else '↺ left'} {abs(d):.0f}° turn → {target}°")
        self.turn_by(d)

    def calibrate(self, keys):
        """90° 회전(time_90 초)을 실제로 해 보고, 돈 각도를 입력받아 출력을 자동 조정 (출력 100% 면 시간을 늘림).
        90° 에 가까워질 때까지 c 를 반복하면 됩니다."""
        print(f"\n[calibration] Mark the car's front direction on the floor. It turns right at {self.duty:.0f}% output for {self.time_90:g} s.")
        time.sleep(1.5)
        self.spin_for(1, self.time_90)
        keys.normal()
        try:
            deg = float(input("Enter the angle it actually turned (deg, 0 if it did not turn): ").strip())
            if deg <= 2:
                self.duty = min(100.0, self.duty + 10)
                print(f"It did not turn, output raised to {self.duty:.0f}%. Press c again")
            else:
                new = DEAD_DUTY + (self.duty - DEAD_DUTY) * 90.0 / deg
                if self.duty >= 100 and (new > 100 or self.time_90 > TIME_90):   # 최대 출력: 이후는 시간만 조정
                    self.time_90 = round(self.time_90 * 90.0 / deg, 2)
                    print(f"{deg:.0f}° even at 100% output → 90° time raised to {self.time_90:g} s (repeat c until 85-95°)")
                else:
                    self.duty = max(DEAD_DUTY + 3, min(100.0, new))
                    print(f"Turned {deg:.0f}° → output set to {self.duty:.0f}% (repeat c until 85-95°)")
            with open(CALIB_FILE, "w") as f:
                json.dump({"duty": self.duty, "time_90": self.time_90}, f)
            self.heading = (self.heading + max(deg, 0)) % 360
        except ValueError:
            print("Not a number, not calibrated")
        keys.raw()
        print(f"Current heading about {self.heading:.0f}° - press 5 to return to 0°")

    # ---------------- 리프트 ----------------
    def _load_lift(self):
        try:
            with open(LIFT_STATE) as f:
                a = float(json.load(f)["angle"])
            if min(DOWN_ANGLE, UP_ANGLE) - 1 <= a <= max(DOWN_ANGLE, UP_ANGLE) + 1:
                return a
        except Exception:
            pass
        return float(DOWN_ANGLE)

    def _save_lift(self):
        try:
            with open(LIFT_STATE, "w") as f:
                json.dump({"angle": round(self.lift, 1)}, f)
        except OSError:
            pass

    def lift_to(self, target):
        if abs(target - self.lift) < 0.5:
            print("Already at that position")
            return
        start = self.lift
        steps = max(1, int(abs(target - start) / LIFT_SPEED / 0.02))
        if SIM:
            print(f"  [sim] lift servo {start:.0f}° → {target}° ({steps * 0.02:.1f} s)")
        try:
            for i in range(1, steps + 1):
                t = i / steps
                t = t * t * (3 - 2 * t)                      # 부드럽게 출발·정지
                self.lift = start + (target - start) * t     # 매 순간 위치 기록 (중간에 멈춰도 정확)
                lgpio.tx_servo(self.h, LIFT_PIN, lift_pulse(self.lift), 50)
                time.sleep(0.02)
            self.lift = float(target)
        finally:
            self._save_lift()

    def close(self):
        self.stop()
        self.station_to("top")
        if round(self.heading) % 360:
            print("To 0°")
            self.turn_to(0)
        if abs(self.lift - DOWN_ANGLE) > 0.5:
            print("Lowering the lift...")
            self.lift_to(DOWN_ANGLE)
        time.sleep(0.5)
        lgpio.tx_servo(self.h, LIFT_PIN, 0)
        if STBY is not None:
            lgpio.gpio_write(self.h, STBY, 0)
        lgpio.gpiochip_close(self.h)


def command(car, words, table):
    """로봇 연동 명령 한 줄 → 응답 dict"""
    try:
        if words[0] == "heading":
            car.turn_to(int(float(words[1])) % 360)
        elif words[0] == "turn":
            car.turn_by(float(words[1]))
        elif words[0] == "lift":
            if words[1] in ("up", "down"):
                car.lift_to(UP_ANGLE if words[1] == "up" else DOWN_ANGLE)
            elif table:
                car.lift_to(round(interp(float(words[1]), table[1], table[0]), 1))
            else:
                raise ValueError("No lift_table.json, mm targets are unavailable (python -m repair_car.lift.turnlift table)")
        elif words[0] == "station" and words[1] in ("top", "front"):
            car.station_to(words[1])
        elif words[0] == "drive":
            car.drive_by(float(words[1]))
        elif words[0] not in ("status", "quit"):
            raise ValueError(f"Unknown command: {words[0]}")
        reply = {"ok": True, "heading": round(car.heading, 1), "station": car.station, "lift_deg": round(car.lift, 1)}
        if table:
            reply["lift_mm"] = round(interp(car.lift, *table), 1)
        reply["quit"] = words[0] == "quit"
        return reply
    except (IndexError, ValueError) as error:
        return {"ok": False, "error": str(error) or "An argument is required", "quit": False}


def serve(car, port):
    """TCP 한 줄 명령 서버: 연결 하나씩 처리, quit 으로 종료(종료 시 0° 복귀·리프트 내림)"""
    table = lift_table()
    with socket.create_server(("0.0.0.0", port)) as server:
        print(f"Waiting for commands: TCP {port} (heading / turn / lift / station / drive / status / quit)")
        while True:
            conn, peer = server.accept()
            print(f"Connected: {peer[0]}")
            with conn, conn.makefile("rw", encoding="utf-8") as stream:
                for line in stream:
                    if not line.split():
                        continue
                    reply = command(car, line.split(), table)
                    stream.write(json.dumps(reply, ensure_ascii=False) + "\n")
                    stream.flush()
                    if reply["quit"]:
                        return


def main():
    parser = argparse.ArgumentParser(description="4-wheel vehicle in-place turn + lift")
    parser.add_argument("--serve", type=int, metavar="PORT", help="TCP command server for the robot (instead of the keyboard)")
    options = parser.parse_args()
    if SIM:
        print("※ No lgpio: simulated run (motors and servos do not move, shown on screen only)")
    car = Car()
    if options.serve:
        try:
            serve(car, options.serve)
        except KeyboardInterrupt:
            print("\nCtrl+C")
        finally:
            car.close()
            print("Quit")
        return
    print("5/6/7/8 = 0°/90°/180°/270° | 3/4 = left/right 90° | 1/2 lift | c calibrate | q quit")
    try:
        with Keys() as keys:
            while True:
                k = keys.get().lower()
                if k in HEADINGS:
                    car.turn_to(HEADINGS[k])
                elif k == "3":
                    print("Left 90°")
                    car.turn_by(-90)
                elif k == "4":
                    print("Right 90°")
                    car.turn_by(90)
                elif k == "1":
                    print("Lift up")
                    car.lift_to(UP_ANGLE)
                elif k == "2":
                    print("Lift down")
                    car.lift_to(DOWN_ANGLE)
                elif k == "c":
                    car.calibrate(keys)
                elif k == "q":
                    break
                else:
                    continue
                if k in "345678":
                    print(f"Current heading about {car.heading:.0f}°")
                keys.flush()
    except KeyboardInterrupt:
        print("\nCtrl+C")
    finally:
        car.close()
        print("Quit")


if __name__ == "__main__":
    main()
