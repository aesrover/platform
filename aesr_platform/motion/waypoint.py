from threading import Thread
import time

from typing import Callable, Union

from aesrdatabaselib.base.waypoint import WaypointManager
from aesrdevicelib.other.timed_motor import TimedControlledMotor


class WaypointSelector(Thread):
    """ Manages selecting and waiting at targets, along with moving the payload. """

    def __init__(self, wpm: WaypointManager, depth_m: TimedControlledMotor):
        super().__init__(target=self.loop)
        self.wpm = wpm
        self.depth_m = depth_m

        self.last_wp = None
        self.is_auto = False
        self.on_target = None
        self.running = False
        self.start()

    def loop(self):
        """ Main loop. """
        while self.running:
            if self._get_auto() is False:
                time.sleep(0.1)
                continue

            if self.get_current_wp() is None:
                self.wpm.select_next_wp()

            if self.on_target is True and self.get_current_wp() is not None:
                wait = self.get_current_wp()['time']
                depth = self.get_current_wp()['depth']

                # Move to desired depth:
                self.depth_m.move_abs_tick(depth, t=True)

                ts = time.time()
                while (time.time() - ts) < wait and self._get_auto():
                    time.sleep(0.05)

                # Get next:
                self.last_wp = self.get_current_wp()
                new_wp = self.wpm.select_next_wp()
                if new_wp is not None:
                    # Move back to starting position if moving to new position for next waypoint
                    if not self.last_wp['pos'] == new_wp['pos']:
                        self.depth_m.move_abs_tick(0, t=True)

                self.on_target = False
            time.sleep(0.5)

    def start(self):
        """ Start thread. """
        self.wpm.select_next_wp()
        self.on_target = False
        self.running = True
        super().start()

    def set_on_wp(self):
        """ Signal that rover is at wp. """
        self.on_target = True

    def get_current_wp(self):
        return self.wpm.get_curr_wp()

    def set_auto(self, state: Union[bool, Callable[[], bool]]):
        self.is_auto = state

    def _get_auto(self):
        if callable(self.is_auto):
            return self.is_auto()
        return self.is_auto

    def close(self):
        self.running = False
        self.join()
