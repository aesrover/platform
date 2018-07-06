import logging
import math

from pymongo import MongoClient
from log4mongo.handlers import MongoHandler

from aesrdatabaselib.main_util import generateTimeName

# Typing:
from typing import List

import RPi.GPIO as GPIO

from aesrdevicelib.motion.blue_esc import BlueESC_PCA9685
from aesrdevicelib.motion.pca9685 import PCA9685
from aesrdevicelib.sensors.bno055 import BNO055, OPERATION_MODE_COMPASS
from aesrdevicelib.sensors.tsys01 import TSYS01
from aesrdevicelib.sensors.ms5837 import MS5837_30BA
from aesrdevicelib.sensors.vernier_odo import VernierODO
from aesrdevicelib.sensors.bme280 import BME280
from aesrdevicelib.sensors.voltage_monitor import VoltageMonitorADS1115
from aesrdevicelib.other.tca9548a import TCA9548A
from aesrdevicelib.motion.md10c import MD10C

from aesrdevicelib.sensors.gps_read import GPSRead
from aesrdatabaselib.mongo.waypoint import MongoWaypointManager
from aesrdevicelib.other.timed_motor import TimedControlledMotor
from aesrdatabaselib.mongo.data import MongoDataHandler

from aesr_platform.motion.waypoint import WaypointSelector
from aesr_platform.motion.movement.movement import ThrusterManager
from aesr_platform.motion.movement.auto import AutoCalc, GPSScaleDiff
from aesr_platform.motion.movement.movement import AutoThrustThreaded
from aesr_platform.motion.movement.control_server.control_server import ControlServer
from aesr_platform.data.store import StoreThreaded
import time

# AUTO CONFIG:
AUTO_INIT_DB = 3
AUTO_HOLD_DB = 5

db_name = generateTimeName()
print("Using Database: {}".format(db_name))

MONGO_HOST = {'host': 'localhost', 'port': 27017}

mongo_db = MongoClient(**MONGO_HOST)[db_name]

log = logging.getLogger('AESR')
log.setLevel(logging.DEBUG)
log.addHandler(logging.StreamHandler())  # Prints to console
log.addHandler(MongoHandler(database_name=db_name, collection='log', **MONGO_HOST))
log.info("Initialized logger.", extra={'type': 'START'})
pca = PCA9685(0x40)
pca.set_pwm_freq(300)
ths = [
    BlueESC_PCA9685((0, 1), 0, 12, pca)
    , BlueESC_PCA9685((-1, 0), math.pi / 2, 13, pca)
    , BlueESC_PCA9685((1, 0), math.pi / 2, 14, pca)
    #, BlueESC_PCA9685((0, -1), 0, 15, pca)
]

# Depth Motor setup:
GPIO.setmode(GPIO.BCM)
depth_m = MD10C(pca.get_channel(11), 5)
tcm = TimedControlledMotor(depth_m, 0.75)

wpm = MongoWaypointManager(mongo_db['wps'])

ws = WaypointSelector(wpm, tcm)

pt = GPSRead()

ht = BNO055(mode=OPERATION_MODE_COMPASS)
try:
    print("BEGIN IMU CALIBRATION:")
    s = (0,) * 4
    #while s[3] != 3:
    while True:
        try:
            s = ht.get_calibration_status()
        except:
            print("IMU failure")
        else:
            print("\rIMU Calibration Status: {}".format(s), end='', flush=True)
        time.sleep(0.1)
except KeyboardInterrupt:
    print("\nSkipping IMU CALIBRATION")


if False:
    tca = TCA9548A()
    transducers = [
        VoltageMonitorADS1115(0, 0x49, resistor_p=336, resistor_n=100),
        TSYS01(pre_func=tca.select_channel, pre_func_args=(0,)),
        MS5837_30BA(pre_func=tca.select_channel, pre_func_args=(0,)),
        VernierODO(),
        BME280(pre_func=tca.select_channel, pre_func_args=(2,))
        , pt
    ]
else:
    transducers = []
tm = ThrusterManager(ths)

ac = AutoCalc(log, pt, ht, 7, AUTO_INIT_DB, AUTO_HOLD_DB, 0.15, min_m=0.5, rot_gain=1.5, dynamic_scale=GPSScaleDiff())

#st = StoreThreaded(log, MongoDataHandler(mongo_db['data']), transducers)
thread_a = AutoThrustThreaded(log, tm, ac, ws, rate=5, t_name="ThrusterCTL")


if __name__ == "__main__":
    try:
        #st.start()
        cs = ControlServer('0.0.0.0', 8000, thread_a)
        cs.run_server()
    except:
        tm.disable_thrusters()
        tcm.close()
        ws.close()
        #st.close()
        pt.close()
