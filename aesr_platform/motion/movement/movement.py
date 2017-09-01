from typing import Tuple, List
import numpy as np
import math
import time
from logging import Logger
from aesrdevicelib.base.motor import Thruster

from threading import Thread
from aesrdevicelib.base.motor import Thruster
from aesrdatabaselib.base.waypoint import WaypointManager

# Local:
from .auto import AutoCalc


class ThrusterManager:
    def __init__(self, thrusters: List[Thruster]):
        self._ths = thrusters
        self._gen_coeff_mat()

    def _gen_coeff_mat(self, lower_bound=10**-10):  # Generate coefficient matrix
        """
        Generate coefficient matrix (and inverse) in to use in thrust calculation.

        T = A*M,
        Where T is the input vector (vx,vy,vr), A is this matrix, and M is the motor powers (m1,m2,m3,m4).

        :param lower_bound:
        :return:
        """

        c_x = []
        c_y = []
        c_rot = []
        for t in self._ths:
            a = t.a
            v = t.v
            c_x.append(math.cos(a))
            c_y.append(math.sin(a))

            center_dist = math.sqrt(v[0] ** 2 + v[1] ** 2)

            # Calculate perpendicular angle of thruster position (pointing counter clockwise):
            perp_ang = math.atan2(v[1], v[0])  # find vector angle
            perp_ang += math.pi / 2  # rotate to perpendicular
            perp_ang %= (2 * math.pi)

            c_rot.append(math.cos(a - perp_ang)
                         * center_dist)

        self.coeff = np.mat([c_x, c_y, c_rot])  # Coefficient matrix

        self.coeff_inv: np.matrix = np.linalg.pinv(self.coeff)

        for (x, y), val in np.ndenumerate(self.coeff_inv):  # filter out small values
            if abs(val) < lower_bound:
                self.coeff_inv[x, y] = 0

    def set_thrust(self, vx: float, vy: float, vr: float) -> None:
        """
        Set thrust of motors from desired movement. Calculate using inverse coefficient matrix.

        :param vx: x axis input
        :param vy: y axis input
        :param vr: z axis rotation input
        :return: None
        """
        thrusts = self.coeff_inv * np.array([[vx], [vy], [vr]])

        # Scale all if above one:
        if len(thrusts) > 0:
            max_t = abs(max(thrusts.max(), thrusts.min(), key=abs))
            if max_t > 1:
                thrusts /= max_t

        for i, t in enumerate(self._ths):
            p = thrusts[i]
            t.set_power(p)

    def disable_thrusters(self):
        for t in self._ths:
            t.set_power(0)


class AutoThrustThreaded(Thread):
    def __init__(self, log: Logger, tm: ThrusterManager, ac: AutoCalc=None, wpm: WaypointManager=None, rate: int=5,
                 t_name: str=None):
        super().__init__(name=t_name)
        self.log = log
        self.tm = tm
        self.ac = ac
        self.rt = 1./rate  # Calculate run time

        self.wpm = wpm

        self.auto = False

        self.running = False
        self.man_t = (0,)*3  # Manual thrust

        self.since_log = None
        self.last_auto = False

    def start(self):
        self.running = True
        super().start()

    def close(self, timeout: float=None):
        self.running = False
        self.join(timeout)

    def __auto_calc(self):
        if self.ac is None:
            raise ValueError("No AutoCalc supplied.")

    def set_auto_state(self, state: bool):
        self.__auto_calc()
        if state:
            self.man_t = None
        else:
            self.man_t = (0,)*3
        self.auto = state

    def set_auto_target(self, t: Tuple[float, float]):
        self.__auto_calc()
        self.ac.set_target(t)

    def next_auto_target(self):
        if self.wpm is None:
            raise ValueError("No waypoint manager!")
        self.__auto_calc()
        self.set_auto_target(self.wpm.select_next_wp())

    def set_thrust(self, vx, vy, vr):
        self.man_t = (vx, vy, vr)

    def run(self):
        while self.running:
            ts = time.time()

            tv = (0,)*3

            curr_pos = None
            curr_ang = None
            mode = None
            extra = {}
            if self.ac is not None and self.auto:
                d = self.ac.calc()
                tv = d.nav
                if not self.last_auto:
                    self.last_auto = True
                    self.log.debug("Auto enabled", extra={'type': 'AUTO', 'itype': 'STATUS', 'status': True,
                                                          'mode': "AUTO"})

                # Set data for log:
                curr_pos = d.pos
                curr_ang = d.ang
                mode = d.mode
                extra['target'] = d.target
                extra['dist'] = d.dist
                extra['diff'] = d.diff

            elif self.man_t is not None:  # Manual
                mode = "MANUAL"
                if self.last_auto:
                    self.last_auto = False
                    self.log.debug("Auto disabled", extra={'type': 'AUTO', 'itype': 'STATUS', 'status': False,
                                                           'mode': "MANUAL"})
                tv = self.man_t

            if self.since_log is None or self.since_log > 10:
                if curr_pos is None:
                    curr_pos = self.ac.read_pt()
                if curr_ang is None:
                    curr_ang = self.ac.read_ht()

                self.since_log = 0

                data = {'type': 'AUTO', 'itype': 'DATA', 'mode': mode, 'tv': tv,
                        'curr_pos': curr_pos, 'curr_ang': curr_ang}
                data.update(**extra)
                self.log.debug("Auto Data", extra=data)
            else:
                self.since_log += 1

            self.tm.set_thrust(*tv)

            if (time.time()-ts) > self.rt:  # Detect
                print("{} dragging".format(self.getName()))
            else:
                while (time.time()-ts) < self.rt:
                    time.sleep(0.0001)
