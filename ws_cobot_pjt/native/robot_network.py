"""Expose the Mac TCP endpoint to DRFL when Docker Desktop translates addresses."""
from pathlib import Path
import json
import selectors
import socket
import struct
import subprocess
import threading


class RobotNetwork:
    def __init__(self, host, token):
        self.host = socket.gethostbyname(host)
        self.source = f'/tmp/cobot-network-{token}.c'
        self.library = f'/tmp/cobot-network-{token}.so'
        self.closed = threading.Event()
        self.server = None
        self.thread = None
        self.workers = []
        self.probe = self.library.removesuffix('.so') + '-probe'

    def start(self):
        subprocess.run(['docker', 'cp', str(Path(__file__).with_name('robot_network_shim.c')),
                        f'ws_cobot_pjt:{self.source}'], check=True)
        subprocess.run(['docker', 'exec', 'ws_cobot_pjt', 'gcc', '-shared', '-fPIC',
                        '-O2', '-Wall', '-Werror', self.source, '-ldl', '-o', self.library],
                       check=True)
        self.server = socket.socket()
        self.server.bind(('127.0.0.1', 0))
        self.server.listen(4)
        self.server.settimeout(0.25)
        self.thread = threading.Thread(target=self.accept, daemon=True)
        self.thread.start()
        return {'COBOT_NETWORK_SHIM': self.library, 'COBOT_ROBOT_IP': self.host,
                'COBOT_ROBOT_RELAY_PORT': str(self.server.getsockname()[1])}

    def check_connection(self, environment, rt_host):
        source = Path(__file__).with_name('robot_connection_probe.cpp')
        subprocess.run(['docker', 'cp', str(source), f'ws_cobot_pjt:{self.probe}.cpp'],
                       check=True)
        common = '/workspace/src/cobot_rg2/doosan-robot2/dsr_common2'
        subprocess.run(['docker', 'exec', 'ws_cobot_pjt', 'g++', '-O2', '-std=c++17',
                        '-DDRCF_VERSION=2', f'-I{common}/include', f'{self.probe}.cpp',
                        f'{common}/lib/jazzy/arm64/libDRFL.a', '-lPocoNet',
                        '-lPocoFoundation', '-lpthread', '-o', self.probe], check=True)
        command = ['docker', 'exec', '-e', f'LD_PRELOAD={self.library}']
        for key, value in environment.items():
            command.extend(['-e', f'{key}={value}'])
        result = subprocess.run([*command, 'ws_cobot_pjt', 'timeout', '12',
                                 self.probe, self.host, rt_host], text=True,
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        print(result.stdout, end='', flush=True)
        result.check_returncode()
        line = next(line for line in result.stdout.splitlines()
                    if line.startswith('COBOT_CONNECTION_RESULT='))
        return json.loads(line.split('=', 1)[1])

    def accept(self):
        while not self.closed.is_set():
            try:
                client, _ = self.server.accept()
            except socket.timeout:
                continue
            except OSError:
                if self.closed.is_set():
                    return
                raise
            worker = threading.Thread(target=self.relay, args=(client,), daemon=True)
            self.workers.append(worker)
            worker.start()

    def relay(self, client):
        try:
            with client, socket.create_connection((self.host, 12345), timeout=5) as robot:
                client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                robot.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                address, port = robot.getsockname()
                # DRFL computes its own authentication from this actual TCP endpoint.
                client.sendall(struct.pack('!4sH', socket.inet_aton(address), port))
                robot.settimeout(None)
                with selectors.DefaultSelector() as selector:
                    selector.register(client, selectors.EVENT_READ, robot)
                    selector.register(robot, selectors.EVENT_READ, client)
                    while not self.closed.is_set():
                        for event, _ in selector.select(timeout=0.25):
                            data = event.fileobj.recv(65536)
                            if not data:
                                return
                            event.data.sendall(data)
        except OSError as error:
            if not self.closed.is_set():
                print(f'Robot TCP relay: {error}', flush=True)

    def close(self):
        self.closed.set()
        if self.server:
            self.server.close()
        if self.thread:
            self.thread.join(timeout=1)
        for worker in self.workers:
            worker.join(timeout=1)
        subprocess.run(['docker', 'exec', 'ws_cobot_pjt', 'rm', '-f',
                        self.source, self.library, self.probe, f'{self.probe}.cpp'],
                       check=False)
