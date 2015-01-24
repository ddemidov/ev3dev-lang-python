#!/usr/bin/python
#
# Controls a robot with attached infrared and touch sensors and
# driven by two large motors.
#
# Moves forward until an object is met, then turns and
# continues to run in the other direction.
# Halts when touch sensor is activated.

import time
from ev3dev import *
from ev3dev_utils.motors import *

lmotor = large_motor(OUTPUT_C)
rmotor = large_motor(OUTPUT_B)
irsens = infrared_sensor()

irsens.mode = infrared_sensor.mode_proximity

ts = touch_sensor()

while not ts.value():
    distance = irsens.value()
    if distance < 0: break

    if distance > 40:
        drive_for(ever=True, left=lmotor, right=rmotor)
        time.sleep(0.01)
    else:
        while irsens.value() < 70 and not ts.value():
            drive_for(ever=True, left=lmotor, right=rmotor, dir=-100)
            time.sleep(0.01)
