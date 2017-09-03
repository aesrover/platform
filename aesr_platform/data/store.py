from threading import Thread
from logging import Logger
import time

# Typing Hints:
from typing import List
from aesrdevicelib.base.transducer import Transducer

from aesrdatabaselib.base.data import DataHandler


class Store:
    def __init__(self, log: Logger, dh: DataHandler, transducers: List[Transducer]):
        self.log = log
        self.transducers = transducers
        self.handler = dh

    def store(self):
        data = []
        for t in self.transducers:
            try:
                d = t.read_full()
            except:
                self.log.exception(t._add_transducer_info({'state': False}))
            else:
                data.append(d)
        self.handler.store_many(data)


class StoreThreaded(Store, Thread):
    def __init__(self, log: Logger, dh: DataHandler, transducers: List[Transducer], store_delay=1, **thread_kwargs):
        super().__init__(log, dh, transducers)
        Thread.__init__(self, **thread_kwargs)
        self.store_delay = store_delay
        self.running = False

    def start(self):
        self.running = True
        Thread.start(self)

    def run(self):
        while self.running:
            t = time.time()
            self.store()
            while (time.time()-t) < self.store_delay:
                time.sleep(0.01)

    def close(self):
        self.running = False
        self.join()
