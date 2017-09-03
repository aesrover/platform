import logging
import math

from log4mongo.handlers import MongoHandler

from aesrdatabaselib.main_util import generateTimeName

# Typing:
from typing import List


from aesrdevicelib.motion.blue_esc import BlueESC_PCA9685
from aesrdevicelib.motion.pca9685 import PCA9685
from aesrdevicelib.sensors.bno055 import BNO055
from aesrdevicelib.sensors.gps_read import GPSRead

from aesr_platform.motion.movement.movement import ThrusterManager
from aesr_platform.motion.movement.auto import AutoCalc, GPSScaleDiff
from aesr_platform.motion.movement.movement import AutoThrustThreaded
from aesr_platform.motion.movement.control_server.control_server import ControlServer
import time

# AUTO CONFIG:
AUTO_INIT_DB = 3
AUTO_HOLD_DB = 5

db_name = generateTimeName()
print("Using Database: {}".format(db_name))

MONGO_HOST = {'host': 'localhost', 'port': 27017}

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


tm = ThrusterManager(ths)
ht = BNO055()
try:
    print("BEGIN IMU CALIBRATION:")
    s = (0,) * 4
    while s[3] != 3:
        s = ht.get_calibration_status()
        print("\rIMU Calibration Status: {}".format(s), end='', flush=True)
        time.sleep(0.1)
except KeyboardInterrupt:
    print("\nSkipping IMU CALIBRATION")


pt = GPSRead()
ac = AutoCalc(log, pt, ht, 10, AUTO_INIT_DB, AUTO_HOLD_DB, 1, min_m=0, rot_gain=0.7, dynamic_scale=GPSScaleDiff())
thread_a = AutoThrustThreaded(log, tm, ac, rate=10, t_name="ThrusterCTL")

if __name__ == "__main__":
    try:
        cs = ControlServer('0.0.0.0', 8000, thread_a)
        cs.run_server()
    except:
        tm.disable_thrusters()
