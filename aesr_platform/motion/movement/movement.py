from typing import Tuple, List
import numpy as np
import math
import time
from aesrdevicelib.base.motor import Thruster

from threading import Thread
from aesrdevicelib.base.motor import Thruster

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
        for i, t in enumerate(self._ths):
            t.set_power(thrusts[i])

    def disable_thrusters(self):
        for t in self._ths:
            t.set_power(0)


class AutoThrustThreaded(Thread):
    def __init__(self, tm: ThrusterManager, ac: AutoCalc=None, rate: int=5, t_name: str=None):
        super().__init__(name=t_name)
        self.tm = tm
        self.ac = ac
        self.rt = 1./rate  # Calculate run time

        self.auto = False

        self.running = False
        self.man_t = None  # Manual thrust

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

    def set_thrust(self, vx, vy, vr):
        self.man_t = (vx, vy, vr)

    def run(self):
        while self.running:
            ts = time.time()

            tv = (0,)*3
            if self.ac is not None and self.auto:
                d = self.ac.calc()
                tv = d.nav
            elif self.man_t is not None:  # Manual
                tv = self.man_t
            else:
                print("No manual value")

            self.tm.set_thrust(*tv)

            if (time.time()-ts) > self.rt:  # Detect
                print("{} dragging".format(self.getName()))
            else:
                while (time.time()-ts) < self.rt:
                    time.sleep(0.0001)
