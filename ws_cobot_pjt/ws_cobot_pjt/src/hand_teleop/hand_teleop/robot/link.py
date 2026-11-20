"""TCP client to the camera side on the macOS host; reconnects until the camera app is up again."""
import collections
import socket
import threading
import time

from ..protocol import JsonLines


class HandLink:
    def __init__(self, host, port):
        self.address, self.lines = (host, port), None
        self.message, self.received = {'tracked': False, 'ground_id': -1}, 0.0
        self.keys = collections.deque()
        threading.Thread(target=self.run, daemon=True).start()

    def run(self):
        while True:
            try:
                with socket.create_connection(self.address, timeout=2.0) as sock:
                    sock.settimeout(None)
                    self.lines = JsonLines(sock)
                    for message in self.lines:
                        self.message, self.received = message, time.monotonic()
                        if message.get('key'):
                            self.keys.append(message['key'])
            except OSError:
                pass
            self.lines = None
            time.sleep(1.0)

    @property
    def connected(self):
        return self.lines is not None

    def latest(self):
        """Last camera message and its age (s)."""
        return self.message, time.monotonic() - self.received

    def key(self):
        return self.keys.popleft() if self.keys else None

    def send(self, **message):
        lines = self.lines
        if lines is None:
            return
        try:
            lines.send(message)
        except OSError:
            pass
