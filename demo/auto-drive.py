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

from sys import exit
from time import sleep
from random import randint
from ev3dev import *

lmotor = large_motor(OUTPUT_C)
rmotor = large_motor(OUTPUT_B)
irsens = infrared_sensor()

assert lmotor.connected, "Left motor is not connected!"
assert rmotor.connected, "Right motor is not connected!"
assert irsens.connected, "Infrared sensor is not connected!"

irsens.mode = 'IR-PROX'

lmotor.speed_regulation_enabled = 'on'
rmotor.speed_regulation_enabled = 'on'

motors = [lmotor, rmotor]

ts = touch_sensor()

def done():
    return ts.connected and ts.value()

while not done():
    distance = irsens.value()

    if distance > 40:
        for m in motors:
            m.speed_sp = 900
            m.set_command('run-forever')

        sleep(0.01)
    else:
        dir = 100 * (randint(0, 1) * 2 - 1)
        while irsens.value() < 70 and not done():
            for (m, p) in zip(motors, steering(dir, 900)):
                m.speed_sp = p
                m.set_command('run-forever')

            sleep(0.01)
