from logging import Logger

# Typing:
from typing import List

from aesrdevicelib.base.transducer import Transducer

# Data:
from .data.data import DataHandler

from .data.store import StoreThreaded


class Platform:
    def __init__(self, log: Logger, *args, transducers: List[Transducer]=None):
        if transducers is None:
            self.transducers = []
        else:
            self.transducers = transducers

    def reg_transducer(self, t: Transducer):
        self.transducers.append(t)

    def run(self):
        pass
