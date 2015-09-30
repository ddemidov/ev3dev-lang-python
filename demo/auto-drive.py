#!/usr/bin/python

print(
"""In this demo a robot with two large motors and an infrared sensor drives
autonomously. It drives forward until an obstacle is met (determined by the
infrared sensor), then turns in a random direction and continues.

If a touch sensor is present, the robot may be stopped by pressing the sensor.

Required hardware:
    large motor on output port B
    large motor on output port C
    infrared sensor on any input port

Optional hardware:
    touch sensor on any input port
"""
)

from time   import sleep
from random import choice
from ev3dev import *

lmotor = large_motor(OUTPUT_C); assert lmotor.connected
rmotor = large_motor(OUTPUT_B); assert rmotor.connected
irsens = infrared_sensor();     assert irsens.connected
ts     = touch_sensor()

irsens.mode = 'IR-PROX'

lmotor.speed_regulation_enabled = 'on'
rmotor.speed_regulation_enabled = 'on'

motors = [lmotor, rmotor]

def done():
    return ts.connected and ts.value()

while not done():
    distance = irsens.value()

    if distance > 40:
        # Path is clear
        for m in motors:
            m.run_forever(speed_sp=900)

        sleep(0.01)
    else:
        # Path is blocked, take a random turn
        dir = choice((100, -100))

        while irsens.value() < 70 and not done():
            for (m, p) in zip(motors, steering(dir, 900)):
                m.run_forever(speed_sp=p)

            sleep(0.01)
