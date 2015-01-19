#!/usr/bin/python

import time, atexit
from ev3dev import *

lmotor = large_motor(OUTPUT_C)
rmotor = large_motor(OUTPUT_B)
irsens = infrared_sensor()

irsens.mode = infrared_sensor.mode_proximity

done = False

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

ts = touch_sensor()
check_sensor(ts, 'Touch')

lmotor.regulation_mode = motor.mode_on
lmotor.run_mode = motor.run_mode_forever
lmotor.run()

rmotor.regulation_mode = motor.mode_on
rmotor.run_mode = motor.run_mode_forever
rmotor.run()

def drive(lp, rp):
    lmotor.pulses_per_second_setpoint = lp
    rmotor.pulses_per_second_setpoint = rp

# Drive until touched.
while not ts.value():
    distance = irsens.value()
    if distance < 0: break

    if distance > 50:
        drive(-600, -600)
        time.sleep(0.01)
    else:
        while distance < 70 and not done:
            drive(-600, 600)
            time.sleep(0.01)
            distance = irsens.value()
