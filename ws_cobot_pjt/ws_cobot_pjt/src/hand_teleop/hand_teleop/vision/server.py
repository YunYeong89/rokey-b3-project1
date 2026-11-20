"""TCP server of the camera side; the ROS node in Docker connects through host.docker.internal."""
import socket
import threading

from ..protocol import JsonLines

WAITING = 'WAITING FOR ROBOT'


class TeleopServer:
    def __init__(self, port):
        self.listener = socket.create_server(('127.0.0.1', port))
        self.link, self.request, self.status = None, -1, WAITING
        threading.Thread(target=self.serve, daemon=True).start()

    def serve(self):
        """One robot connection at a time; its grounding requests and status lines. A new connection starts over."""
        while True:
            sock, _ = self.listener.accept()
            self.link = JsonLines(sock)
            try:
                for message in self.link:
                    self.request = message.get('ground', self.request)
                    self.status = message.get('status', self.status)
            except OSError:
                pass
            sock.close()
            self.link, self.request, self.status = None, -1, WAITING

    def publish(self, message):
        link = self.link
        if link is None:
            return
        try:
            link.send(message)
        except OSError:
            self.link = None
