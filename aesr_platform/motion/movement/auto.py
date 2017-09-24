import math
import numpy as np

from logging import Logger

from typing import Union, Tuple

from aesrdevicelib.base.navigation import PositionTransducer, HeadingTransducer


class DynamicScaleDiff:
    @staticmethod
    def _diff(t: tuple, cp: tuple):
        return np.subtract(t, cp)

    def calc(self, t, cp) -> np.ndarray:
        raise NotImplementedError


class GPSScaleDiff(DynamicScaleDiff):
    def calc(self, t, cp):
        return self._diff(t, cp)*np.array((math.cos(math.radians(cp[1])) * 111320., 110540.))


class _Mode:
    REPOS = 1  # Repositioning
    HOLD = 2  # Holding (not driving)


class AutoCalc:
    class AutoData:
        def __init__(self, nav: Tuple[float,float,float], state: Union[None, bool], pos: Tuple[float, float],
                     ang: float, mode: str, target: Tuple[float, float], dist, diff):
            self.nav = nav
            self.state = state
            self.pos = pos
            self.ang = ang
            self.mode = mode
            self.target = target
            self.dist = dist
            self.diff = diff

    def __init__(self, log: Logger, pt: PositionTransducer, ht: HeadingTransducer, max_d, target_r, hold_r, max_m, min_m=0,
                 rot_gain=1, diff_scale: Tuple[float, float] = (1,1), dynamic_scale: DynamicScaleDiff=None):
        """
        :param pt: PositionTransducer used for distance/translation
        :param ht: HeadingTransducer used for current angle
        :param max_d: Maximum distance to begin scaling thruster power
        :param target_r: Initial hold shutdown distance
        :param hold_r: Distance before re-engaging thrusters for repositioning
        :param max_m: Max motor power
        :param min_m: Minimum motor power
        :param rot_gain: Rotation influence gain
        :param diff_scale: X,Y distance scaling (after calculating difference to target)
        :param dynamic_scale: DynamicScaleDiff to dynamically scale X,Y distance (example: lon/lat to x/y meters)
        """
        self.log = log
        self.pt = pt
        self.ht = ht
        self.scale = diff_scale
        self.d_scale = dynamic_scale
        self.max_d = max_d
        self.min_m = min_m
        self.target_r = target_r
        self.hold_r = hold_r
        self.max_m = max_m
        self.rot_g = rot_gain

        self.mode = _Mode.REPOS

        self.target: Tuple[float, float] = None

    #def _pos_diff(self, target: Tuple[float, float]):
    #    curr_pos = self.pt.read_xy_pos()
    #    return tuple(np.subtract(target, curr_pos))

    def set_target(self, t: Tuple[float, float]):
        self.target = t
        self.mode = _Mode.REPOS  # Ensure repositioning is enabled

    def calc(self) -> AutoData:
        if self.target is None:
            raise ValueError("No Target")
            #return self.AutoData((0,0,0), None, )

        curr_pos = self.read_pt()

        a = self.read_ht()

        if self.d_scale is not None:
            pd_dir: np.ndarray = self.d_scale.calc(self.target, curr_pos)
        else:
            pd_dir: np.ndarray = np.multiply(self.scale, tuple(np.subtract(self.target, curr_pos)))

        pd_scaled = []
        for i, v in np.ndenumerate(pd_dir):
            v /= self.max_d
            if abs(v) > 1:  # Limit to 1 (before max motor value is applied)
                v = math.copysign(1, v)
            v *= (self.max_m-self.min_m)
            v += self.min_m * math.copysign(1, v)
            pd_scaled.append(v)

        pd_scaled = np.array(pd_scaled)

        # Calculate distance from target:
        dist = math.hypot(pd_dir[0], pd_dir[1])

        if a is None:
            return self.AutoData((0, 0, 0), False, curr_pos, None, "NOHEADING", self.target, dist, tuple(pd_dir))

        if curr_pos[0] is None or curr_pos[1] is None:
            return self.AutoData((0, 0, 0), False, curr_pos, a, "NOGPS", self.target, dist, tuple(pd_dir))


        # Check if inside target area in repositioning mode
        if self.mode == _Mode.REPOS and dist < self.target_r:
            self.mode = _Mode.HOLD
        # Check if outside hold area:
        if self.mode == _Mode.HOLD and dist > self.hold_r:
            self.mode = _Mode.REPOS

        # If in hold mode, return no thrust:
        if self.mode == _Mode.HOLD:
            return self.AutoData((0, 0, 0), True, curr_pos, a, "HOLD", self.target, dist, tuple(pd_dir))

        # Calculate scaled angle influence for thruster:
        a_r = a/math.pi
        a_r *= -self.rot_g

        # Calculate rotation matrix:
        s = math.sin(-a)
        c = math.cos(-a)
        rot_mat = np.mat([[c, -s], [s, c]])

        # Calculate rotated x,y vector:
        rotv = rot_mat.dot(pd_scaled)

        return self.AutoData((rotv[0, 0], rotv[0, 1], a_r), False, curr_pos, a, "REPOS", self.target, dist, tuple(pd_dir))

    def read_ht(self):
        try:
            a = self.ht.read_heading()
        except:
            self.log.exception("IMU READ FAILURE", extra={'atype': 'IMU', 'state': False})
            return None
        a = ((a + 180) % 360) - 180  # Convert to [-180,180] range
        a *= math.pi / 180  # convert to radians now [-pi,pi] range
        a *= -1  # Flip angle (increases with counter clockwise rotation)
        return a

    def read_pt(self):
        try:
            return self.pt.read_xy_pos()
        except:
            self.log.error("GPS READ FAILURE", extra={'atype': 'GPS', 'state': False})
            return None, None
