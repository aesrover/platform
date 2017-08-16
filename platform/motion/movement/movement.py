from typing import Tuple, List
import numpy as np
import math

# Local:
from .thruster import Thruster


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
