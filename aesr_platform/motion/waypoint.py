from threading import Thread
import time

from aesrdatabaselib.base.waypoint import WaypointManager
from aesrdevicelib.other.timed_motor import TimedControlledMotor


class WaypointSelector(Thread):
    """ Manages selecting and waiting at targets, along with moving the payload. """

    def __init__(self, wpm: WaypointManager, depth_m: TimedControlledMotor):
        super().__init__(target=self.loop)
        self.wpm = wpm
        self.depth_m = depth_m

        self.last_wp = None
        self.curr_wp = None
        self.on_target = None
        self.running = False
        self.start()

    def loop(self):
        """ Main loop. """
        while self.running:
            if self.curr_wp is None:
                self.curr_wp = self.wpm.select_next_wp()

            if self.on_target is True and self.curr_wp is not None:
                wait = self.curr_wp['time']
                depth = self.curr_wp['depth']

                # Move to desired depth:
                self.depth_m.move_abs_tick(depth)

                time.sleep(wait)  # Wait at waypoint

                # Get next:
                self.last_wp = self.curr_wp
                new_wp = self.wpm.select_next_wp()
                if new_wp is not None:
                    # Move back to starting position if moving to new position for next waypoint
                    if not self.last_wp['pos'] == new_wp['pos']:
                        self.depth_m.move_abs_tick(0)

                self.curr_wp = new_wp
                self.on_target = False
            time.sleep(0.5)

    def start(self):
        """ Start thread. """
        self.curr_wp = self.wpm.select_next_wp()
        self.on_target = False
        self.running = True
        super().start()

    def set_on_wp(self):
        """ Signal that rover is at wp. """
        self.on_target = True

    def get_current_wp(self):
        return self.curr_wp

    def close(self):
        self.running = False
        self.join()
