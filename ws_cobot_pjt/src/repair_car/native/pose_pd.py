"""Import the shared controller for the native simulator."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[4] / 'ws_cobot_pjt/src/repair_car'))
from repair_car.robot_control.cartesian_servo import PDSettings, PosePD, VectorPD
