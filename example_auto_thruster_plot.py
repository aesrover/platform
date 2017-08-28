import matplotlib.pyplot as plt
import math
import random
import numpy as np

# Typing:
from typing import Tuple

from aesrdevicelib.base.motor import Thruster
from aesrdevicelib.base.navigation import PositionTransducer, HeadingTransducer

from platform.motion.movement.auto import AutoCalc
from platform.motion.movement.movement import ThrusterManager


class TestPT(PositionTransducer):
    def __init__(self):
        super().__init__("Fake PT")
        self.p = (0,0)

    def set_position(self, vx, vy):
        self.p = (vx, vy)

    def read(self):
        return self.p

    def read_xy_pos(self) -> Tuple[float, float]:
        return self.read()


class TestHT(HeadingTransducer):
    def __init__(self):
        super().__init__("Fake HT")
        self.a = 0

    def set_heading(self, a):
        self.a = a

    def read(self):
        return self.a

    def read_heading(self):
        return self.read()


class FakeThruster(Thruster):
    def __init__(self, num, v, a):
        super().__init__(v, a)
        self.num = num

    def _set_motor_power(self, p: float):
        print("T{} Power: {}".format(self.num, p))


FPS_SCALE = 1/10
LIN_GAIN = 10
ROT_GAIN = 10
TARGETS = [(0,0), (10,5), (5,5)]
ths = ((math.pi/2, (10, 0)), (math.pi/2, (-1, 0)), (0, (0, 1)), (0, (0, -1)))  # All Thrusters

if __name__ == "__main__":
    thrusters = []
    for i, t in enumerate(ths):
        vec = t[1]
        ang = t[0]
        thrusters.append(FakeThruster(i, vec, ang))

    tm = ThrusterManager(thrusters)

    pt = TestPT()
    pt.set_position(10,10)
    ht = TestHT()
    ht.set_heading(90)
    ac = AutoCalc(pt, ht, 10, 1, 5, 0.2, min_m=0.01, rot_gain=0.5)

    t_i = -1

    def next_target(r=True):
        global t_i, ac
        t_i += 1
        try:
            t = TARGETS[t_i]
            ac.set_target(t)
            return t
        except IndexError:
            if r:
                raise
            else:
                return None

    next_target()


    # --- PLOT ---
    # Create figure:
    fig1 = plt.figure(1)
    # Clear figure:
    plt.clf()
    # Enable interactive mode:
    plt.ion()
    # Create subplot:
    sbp = plt.subplot(111)
    sbp.autoscale(True)
    # Create axis:
    ax = fig1.add_axes(sbp)
    plt.axis('equal')  # CRUCIAL for quiver alignment
    print("TYPE: {}".format(type(ax)))

    # Initial plot:
    target_pos, = plt.plot([ac.target[0]], [ac.target[1]], lw=1, marker='+', color='r')  # Target
    platform_pos, = plt.plot([], [], lw=1, marker='+', color='r')

    heading_ap = None
    thrustv_ap = None
    targetv_ap = None

    def arrow_update(ap, x,y,dx,dy, *args, **kwargs):
        if ap is None:
            return ax.quiver(x, y, dx, dy, *args, **kwargs)

        ap.set_offsets((x, y))
        ap.set_UVC(dx, dy)
        return ap


    while True:
        print("\n\n")
        d = ac.calc()
        cur_pos = d.pos
        cur_ang = d.ang
        print("Nav {}, State: {}".format(d.nav, d.state))

        # Update plot data:
        platform_pos.set_xdata([cur_pos[0]])
        platform_pos.set_ydata([cur_pos[1]])
        #tm.set_thrust()

        # Add arrow
        heading_ap = arrow_update(heading_ap, cur_pos[0], cur_pos[1], 10*math.sin(cur_ang), 10*math.cos(cur_ang),
                                  color="#aa0088")

        # Update fake transducers:
        s = math.sin(-cur_ang)
        c = math.cos(-cur_ang)
        rot_mat = np.mat([[c, -s], [s, c]])
        # Calculate rotated x,y vector:
        revv = rot_mat.dot(d.nav[0:2])
        print("Reversed vector: {}".format(revv))
        thrustv_ap = arrow_update(thrustv_ap, cur_pos[0], cur_pos[1], 10*revv[0,0], 10*revv[0,1],
                                  color="#000000", units='inches')# #f1f442

        #targetv_ap = arrow_update(targetv_ap, cur_pos[0], cur_pos[1], ac.target[0]-cur_pos[0],
        #                          ac.target[1]-cur_pos[1], color="#4286f4", units='inches')

        # PLOT UPDATE:
        # Fix autoscale:
        ax.relim()
        ax.autoscale_view()
        # Draw new plot:
        plt.draw()
        # Pause for plot update:
        plt.pause(FPS_SCALE)

        if d.state:
            t = next_target(False)  # Get next target
            if t is None:
                print("\nDONE!")
                break
            target_pos.set_xdata([t[0]])
            target_pos.set_ydata([t[1]])

        pt.set_position(cur_pos[0]+(revv[0,0]*FPS_SCALE*LIN_GAIN), cur_pos[1]+(revv[0,1]*FPS_SCALE*LIN_GAIN))

        print("CURR ANG: {}".format(cur_ang))
        angle_step = (d.nav[2])*FPS_SCALE*ROT_GAIN
        print("STEP ANGLE DIFF: {}".format(angle_step))
        ht.set_heading((((cur_ang-angle_step)*(180/math.pi))+360)%360)
        print("ANG NEW: {}".format(ht.a))
        print("TARGETS: {}, i: {}".format(TARGETS, t_i))
