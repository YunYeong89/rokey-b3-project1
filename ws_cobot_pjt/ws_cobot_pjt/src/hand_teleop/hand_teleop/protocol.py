"""Newline-delimited JSON over one TCP socket between the macOS camera side and the ROS node in Docker.

camera -> robot, every frame (~30 Hz):
  {"tracked": bool, "ground_id": int, "progress": 0..1,
   "offset": [x, y, z] mm, "rotation": [x, y, z, w], "openness": 0..1, "key": "home" | "stop" | "continue" | "quit"}
  offset and rotation are the hand since grounding in camera axes (x right, y down, z away from the lens, raw image);
  "key" only in the frame a key was pressed.
robot -> camera: {"ground": id} starts a grounding with that id, {"status": text} for the overlay.
"""
import json
import threading

PORT = 5070


class JsonLines:
    def __init__(self, sock):
        self.sock, self.reader, self.lock = sock, sock.makefile('r'), threading.Lock()

    def send(self, message):
        with self.lock:
            self.sock.sendall((json.dumps(message) + '\n').encode())

    def __iter__(self):
        for line in self.reader:
            yield json.loads(line)
