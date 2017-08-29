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
        return self._diff(t, cp)*np.array((math.cos(math.radians(cp[0])) * 111320., 110540.))


class _Mode:
    REPOS = 1  # Repositioning
    HOLD = 2  # Holding (not driving)


class AutoCalc:
    class AutoData:
        def __init__(self, nav: Tuple[float,float,float], state: Union[None, bool], pos: Tuple[float, float],
                     ang: float, mode: str):
            self.nav = nav
            self.state = state
            self.pos = pos
            self.ang = ang
            self.mode = mode

    def __init__(self, log: Logger, pt: PositionTransducer, ht: HeadingTransducer, max_d, target_r, hold_r, max_m, min_m=0,
                 rot_gain=1, diff_scale: Tuple[float, float] = (1,1), dynamic_scale: DynamicScaleDiff=None):
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

        try:
            curr_pos = self.pt.read_xy_pos()
        except:
            curr_pos = (None, None)
        print("Cur pos: {}, target: {}".format(curr_pos, self.target))

        try:
            a = self.ht.read_heading()
        except:
            self.log.exception("IMU READ FAILURE", extra={'atype': 'IMU', 'state': False})
            return self.AutoData((0, 0, 0), False, curr_pos, None, mode="NOHEADING")

        a = ((a + 180) % 360) - 180  # Convert to [-180,180] range
        a *= math.pi / 180  # convert to radians now [-pi,pi] range
        print("Angle: {}".format(a))

        if curr_pos[0] is None or curr_pos[1] is None:
            print("NO GPS")
            self.log.error("GPS READ FAILURE", extra={'atype': 'GPS', 'state': False})
            return self.AutoData((0, 0, 0), False, curr_pos, a, mode="NOGPS")

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
            print("V: {}".format(v))
            pd_scaled.append(v)

        pd_scaled = np.array(pd_scaled)

        print("Diffs: {}".format(pd_scaled))

        # Calculate distance from target:
        dist = math.hypot(pd_dir[0], pd_dir[1])
        print("Dist: {}".format(dist))

        # Check if inside target area in repositioning mode
        if self.mode == _Mode.REPOS and dist < self.target_r:
            self.mode = _Mode.HOLD
        # Check if outside hold area:
        if self.mode == _Mode.HOLD and dist > self.hold_r:
            self.mode = _Mode.REPOS

        # If in hold mode, return no thrust:
        if self.mode == _Mode.HOLD:
            return self.AutoData((0, 0, 0), True, curr_pos, a, mode="HOLD")

        # Calculate scaled angle influence for thruster:
        a_r = a/math.pi
        a_r *= self.rot_g

        # Calculate rotation matrix:
        s = math.sin(a)
        c = math.cos(a)
        rot_mat = np.mat([[c, -s], [s, c]])

        # Calculate rotated x,y vector:
        rotv = rot_mat.dot(pd_scaled)
        print("Rotated vector: {}".format(rotv))

        return self.AutoData((rotv[0, 0], rotv[0, 1], a_r), False, curr_pos, a, mode="REPOS")
