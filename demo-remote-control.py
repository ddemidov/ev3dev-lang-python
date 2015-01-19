#!/usr/bin/python

import time, atexit
from ev3dev import *

rmotor = large_motor(OUTPUT_B)
lmotor = large_motor(OUTPUT_C)

def reset():
    rmotor.reset()
    lmotor.reset()

atexit.register(reset)

def check_sensor(s, name):
    if s.connected():
        print("Found %s sensor at %s" % (name, s.port_name()))
    else:
        print("%s sensor is not connected" % name)
        sys.exit(1)

ir = infrared_sensor()
check_sensor(ir, 'Infrared')

ts = touch_sensor()
check_sensor(ts, 'Touch')

rc = remote_control(ir)

def make_roll(m, p):
    def roll(state):
        if state:
            m.regulation_mode = motor.mode_on
            m.run_mode = motor.run_mode_forever
            m.pulses_per_second_setpoint = p
            m.run()
        else:
            m.reset()
    return roll

rc.on_red_up   (make_roll(lmotor, -600))
rc.on_red_down (make_roll(lmotor,  600))
rc.on_blue_up  (make_roll(rmotor, -600))
rc.on_blue_down(make_roll(rmotor,  600))

while not ts.value():
    if not rc.process(): time.sleep(0.01)

