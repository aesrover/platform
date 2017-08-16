# Type Hints:
from typing import Tuple

# Local:
from .transducer import Transducer


class PositionTransducer(Transducer):
    def __init__(self, atype, itype=None, **other):
        super().__init__(atype, itype=itype, **other)

    def read_transducer(self):
        raise NotImplementedError

    def read_xy_pos(self) -> Tuple[float, float]:
        raise NotImplementedError


class HeadingTransducer(Transducer):
    def __init__(self, atype, itype=None, **other):
        super().__init__(atype, itype=itype, **other)

    def read_transducer(self):
        raise NotImplementedError

    def read_heading(self) -> float:
        raise NotImplementedError
