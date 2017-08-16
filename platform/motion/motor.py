class Motor:
    def _set_motor_power(self, p: float):
        raise NotImplementedError

    def set_power(self, p: float):
        if abs(p) > 1:
            raise ValueError("Power value needs to be in range [-1,1].")
        return self._set_motor_power(p)
