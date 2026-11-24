"""TCP client of the firmware's command server (firmware/car_turn_lift.py --serve): one line out, one JSON line back,
one connection per command (the server takes one connection at a time, so the ROS bridge and a terminal can share it).
python -m repair_car.lift.firmware [--host H] [--port P] COMMAND...  sends one command and prints the reply."""
import argparse
import json
import socket


class FirmwareClient:
    def __init__(self, host, port, timeout=60.):
        self.address, self.timeout = (host, port), timeout

    def send(self, line):
        with socket.create_connection(self.address, self.timeout) as conn, \
                conn.makefile('rw', encoding='utf-8') as stream:
            stream.write(line.strip() + '\n')
            stream.flush()
            return json.loads(stream.readline())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=5005)
    parser.add_argument('command', nargs='+', help='heading DEG | turn DEG | lift up|down|MM | station top|front | '
                                                   'drive MM | status | quit')
    options = parser.parse_args()
    print(json.dumps(FirmwareClient(options.host, options.port).send(' '.join(options.command)), ensure_ascii=False))


if __name__ == '__main__':
    main()
