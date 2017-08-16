# Typing Hints:
from typing import Tuple, List

from ..motor import Motor


class Thruster(Motor):
    def __init__(self, v: Tuple[float, float], a: float):
        self.v = v
        self.a = a

    def _set_motor_power(self, p: float):
        raise NotImplementedError
