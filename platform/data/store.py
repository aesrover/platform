from threading import Thread
from logging import Logger
import time

# Typing Hints:
from typing import List
from aesrdevicelib.base.transducer import Transducer

from .data import DataHandler


class Store:
    def __init__(self, log: Logger, transducers: List[Transducer]):
        self.transducers = transducers
        self.handlers = []

    def add_handler(self, handler: DataHandler):
        self.handlers.append(handler)

    def store(self):
        for t in self.transducers:
            t.read()


class StoreThreaded(Store, Thread):
    def __init__(self, log: Logger, transducers: List[Transducer], store_delay=1, **thread_kwargs):
        super().__init__(log, transducers)
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
            while not (time.time()-t) < self.store_delay:
                time.sleep(0.01)

    def close(self):
        self.running = False
        self.join()
