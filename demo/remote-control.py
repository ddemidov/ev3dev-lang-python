#!/usr/bin/python
#
# Remotely controls a robot with infrared and touch sensor attached
# and driven by two large motors.
#
# Halts when touch sensor is activated.

import time
from ev3dev import *
from ev3dev_utils.motors import *

rmotor = large_motor(OUTPUT_B)
lmotor = large_motor(OUTPUT_C)

ir = infrared_sensor()
ts = touch_sensor()
rc = remote_control(ir)

def make_roll(m, p):
    def roll(state):
        if state:
            run_for(ever=True, motor=m, power=p)
        else:
            m.reset()
    return roll

rc.on_red_up   (make_roll(lmotor,  80))
rc.on_red_down (make_roll(lmotor, -80))
rc.on_blue_up  (make_roll(rmotor,  80))
rc.on_blue_down(make_roll(rmotor, -80))

while not ts.value():
    if not rc.process(): time.sleep(0.01)

