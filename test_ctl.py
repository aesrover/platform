from pymongo import MongoClient
import logging
import time
from typing import Tuple

from aesr_platform.motion.movement.movement import ThrusterManager
from aesr_platform.motion.movement.control_server.control_server import ControlServer
from aesrdevicelib.base.navigation import PositionTransducer, HeadingTransducer

from aesr_platform.motion.movement.auto import AutoCalc, GPSScaleDiff
from aesr_platform.motion.movement.movement import AutoThrustThreaded

from aesr_platform.motion.waypoint import WaypointSelector
from aesrdatabaselib.mongo.waypoint import MongoWaypointManager
from aesrdevicelib.other.timed_motor import TimedControlledMotor
from aesrdevicelib.base.motor import Motor, Thruster

log = logging.getLogger('AESR')
log.setLevel(logging.DEBUG)
log.addHandler(logging.StreamHandler())  # Prints to console
log.info("Initialized logger.", extra={'type': 'START'})


class FakeMotor(Motor):
    def _set_motor_power(self, p: float):
        print("Motor to: {}".format(p))


class FakeThuster(Thruster):
    def _set_motor_power(self, p: float):
        print("Motor to: {}".format(p))


class TestPT(PositionTransducer):
    def __init__(self):
        super().__init__("Fake PT")
        self.p = (0, 0)

    def set_position(self, vx, vy):
        self.p = (vx, vy)

    def read(self):
        return self.p

    def read_xy_pos(self) -> Tuple[float, float]:
        return self.read()


class TestHT(HeadingTransducer):
    def __init__(self):
        super().__init__("Fake HT")
        self.a = 0

    def set_heading(self, a):
        self.a = a

    def read(self):
        return self.a

    def read_heading(self):
        return self.read()


# AUTO CONFIG:
AUTO_INIT_DB = 3
AUTO_HOLD_DB = 5

ths = [
    FakeThuster((0,1), 0),
    FakeThuster((0,-1), 0)
]

fm = FakeMotor()

tm = ThrusterManager(ths)

tcm = TimedControlledMotor(fm, -0.5)

col = MongoClient()['wps_test2']['wps']
wpm = MongoWaypointManager(col)

ws = WaypointSelector(wpm, tcm)

pt = TestPT()
pt.set_position(-71.325143, 41.735677)
ht = TestHT()
ac = AutoCalc(log, pt, ht, 10, AUTO_INIT_DB, AUTO_HOLD_DB, 1, min_m=0, rot_gain=0.7, dynamic_scale=GPSScaleDiff())
thread_a = AutoThrustThreaded(log, tm, ac, ws, rate=1, t_name="ThrusterCTL")


if __name__ == "__main__":
    try:
        cs = ControlServer('0.0.0.0', 8000, thread_a)
        cs.run_server()
    except:
        tm.disable_thrusters()
        tcm.close()
        ws.close()
