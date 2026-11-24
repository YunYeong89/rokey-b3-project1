"""The unchanged firmware (firmware/car_turn_lift.py) on the Gazebo scissor lift (gz_lift.py): its lgpio calls go to
SimGPIO, and every pulse on the lift servo pin becomes a crank target on gz-transport (crank = servo travel from
DOWN_ANGLE), while its clock stays real (Gazebo runs in real time). The Gazebo model is the lift alone: wheel pins are
only recorded, so turns, stations and drives take their firmware time without motion. serve(port) runs the firmware's
TCP command server, which the ROS bridge (lift/node.py) and lift.firmware call."""
import importlib.util

import numpy as np

from .config import OUT, ROOT
from .gpio import SimGPIO
from .gz_lift import NAME


class GzGPIO(SimGPIO):
    def __init__(self, on_servo):
        super().__init__()
        self.on_servo = on_servo

    def tx_servo(self, h, pin, width, freq=50):
        super().tx_servo(h, pin, width, freq)
        self.on_servo(pin)


class GzFirmware:
    def __init__(self):
        from gz.msgs10.double_pb2 import Double
        from gz.transport13 import Node
        self.node, self.Double = Node(), Double
        self.publisher = self.node.advertise(f'/{NAME}/servo', Double)
        self.gpio = GzGPIO(self.on_servo)
        spec = importlib.util.spec_from_file_location('car_turn_lift_gz', ROOT / 'firmware/car_turn_lift.py')
        self.fw = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.fw)
        folder = OUT / 'gazebo'
        folder.mkdir(parents=True, exist_ok=True)
        self.fw.lgpio, self.fw.SIM = self.gpio, False
        self.fw.LIFT_STATE, self.fw.CALIB_FILE = str(folder / 'lift_state.json'), str(folder / 'turn_calib.json')
        for name in ('lift_state.json', 'turn_calib.json'):      # fresh: the Gazebo lift starts down
            (folder / name).unlink(missing_ok=True)

    def on_servo(self, pin):
        angle = self.gpio.angle(pin)
        if pin != self.fw.LIFT_PIN or angle is None:            # wheel pins, or the servo pulse switched off
            return
        crank = (angle - self.fw.DOWN_ANGLE) * np.sign(self.fw.UP_ANGLE - self.fw.DOWN_ANGLE)
        message = self.Double()
        message.data = float(np.radians(crank))
        self.publisher.publish(message)

    def serve(self, port):
        car = self.fw.Car()
        try:
            self.fw.serve(car, port)
        except KeyboardInterrupt:
            print('\nCtrl+C')
        finally:
            car.close()
