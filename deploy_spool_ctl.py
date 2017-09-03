from aesrdevicelib.motion.pca9685 import PCA9685
from aesrdevicelib.motion.md10c import MD10C
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

pc = PCA9685().get_channel(11)
pc.set_pwm_freq(300)
m = MD10C(pc, 5)

power = 0.5
try:
    while True:
        m.set_power(0)
        time.sleep(0.2)
        i = input("\nPlease input 'W' for up, and 'S' for down: ")
        i = i.lower()
        if i == 'w':
            print("up")
            m.set_power(power)
        elif i == 's':
            print("down")
            m.set_power(-power)
        else:
            print("Invalid input")
            continue
        time.sleep(1)
finally:
    m.set_power(0)
