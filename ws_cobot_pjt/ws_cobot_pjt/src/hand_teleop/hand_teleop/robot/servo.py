"""DSR servol stream: the controller moves toward the latest target (BASE mm, ZYZ deg) within vel/acc. The same
topic drives the DRCF emulator (the Gazebo session follows its joints) and the real M0609."""
from dsr_msgs2.msg import ServolStream
from rokey.rotations import angles_zyz


class ServoStream:
    def __init__(self, node, config):
        self.publisher = node.create_publisher(ServolStream, 'dsr_controller2/servol_stream', 10)
        self.vel, self.acc = [float(v) for v in config['vel']], [float(v) for v in config['acc']]
        self.angles = None

    def reset(self, angles):
        self.angles = list(angles)

    def send(self, position, rotation):
        self.angles = angles_zyz(rotation, self.angles)
        self.publisher.publish(ServolStream(pos=[*map(float, position), *map(float, self.angles)],
                                            vel=self.vel, acc=self.acc, time=0.0))
