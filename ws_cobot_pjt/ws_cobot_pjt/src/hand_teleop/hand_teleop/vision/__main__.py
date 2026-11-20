"""macOS camera side of hand_teleop: webcam -> hand landmarks -> grounding -> TCP to `ros2 run hand_teleop teleop`.

  ./run_camera.sh [--camera 0] [--port 5070] [--seconds 5]
"""
import argparse
import time

from ..protocol import PORT
from .grounding import Grounding
from .overlay import Overlay
from .server import TeleopServer
from .tracker import HandTracker


class CameraApp:
    def __init__(self, args):
        self.tracker = HandTracker(args.model, args.camera)
        self.grounding = Grounding(args.seconds, fist_curl=args.fist_curl, focal_px=args.focal_px)
        self.server = TeleopServer(args.port)
        self.overlay = Overlay()

    def run(self):
        key = None
        while key != 'quit':
            frame, hand = self.tracker.read()
            if self.server.request != self.grounding.id:
                self.grounding.start(self.server.request)
            message = self.grounding.update(hand, time.monotonic())
            key = self.overlay.show(frame, hand, self.grounding, self.server.status)
            self.server.publish({**message, 'key': key} if key else message)
        self.tracker.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--model', required=True, help='MediaPipe hand_landmarker.task')
    parser.add_argument('--camera', type=int, default=0)
    parser.add_argument('--port', type=int, default=PORT)
    parser.add_argument('--seconds', type=float, default=5.0, help='grounding time (s)')
    parser.add_argument('--fist-curl', type=float, default=1.1, help='finger curl of a fist: openness 0')
    parser.add_argument('--focal-px', type=float, default=500.0, help='webcam focal length at 640 x 480 (depth in mm)')
    CameraApp(parser.parse_args()).run()


if __name__ == '__main__':
    main()
