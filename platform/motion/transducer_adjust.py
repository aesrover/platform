import time

# Logging:
from logging import Logger

from aesrdevicelib.base.motor import Motor
from aesrdevicelib.base.transducer import Transducer


class TransducerAdjustHandler:
    def __init__(self):
        pass

    def adjust(self, *a, **k):
        raise NotImplementedError("Method `adjust` should be implemented.")


class TransducerMotorDrive(TransducerAdjustHandler):
    def __init__(self, log: Logger, m: Motor, t: Transducer, max_diff: float, min_pow: float, max_pow: float,
                 reverse: bool=False, timeout=None):
        super().__init__()
        self.log = log
        self.m = m
        self.t = t
        self.max_diff = max_diff
        self.timeout = timeout
        self.min_pow = min_pow
        self.max_pow = max_pow

        # Set gain based on if reversed:
        if reverse:
            self.gain = -1
        else:
            self.gain = 1

    def _adjust_step(self, target) -> bool:
        t_val = self.t.read_transducer()  # Read transducer

        diff = target - t_val  # Calculate difference between target value and current
        p = diff/self.max_diff  # Divide by max difference
        if p > 1:
            p = 1
        elif p < -1:
            p = -1

        p *= self.max_pow  # Scale for maximum power

        if abs(p) < self.min_pow:  # Cut off at minimum power
            self.m.set_power(0)
            return True

        self.m.set_power(p * self.gain)
        return False

    def adjust(self, target) -> bool:
        st = time.time()  # Start time
        while True:
            # If timeout was supplied, check if the timeout time has been reached:
            if self.timeout is not None and time.time()-st > self.timeout:
                return False

            # Run step, and return:
            if self._adjust_step(target):
                self.log.debug("Motor transducer adjust reached target {}, after {} seconds".format(target,
                                                                                                    time.time()-st))
                return True
            time.sleep(0.1)
