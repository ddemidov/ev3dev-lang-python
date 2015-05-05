#!/usr/bin/python

print(
"""This demo shows how to control a robot with a remote.

The robot should have two large motors and an infrared sensor attached.
Red buttons on the remote control the left motor, blue buttons control
the right one.

Required hardware:
    large motor on output port B
    large motor on output port C
    infrared sensor on any input port
"""
)

from time import sleep
from ev3dev import *

lmotor = large_motor(OUTPUT_C)
rmotor = large_motor(OUTPUT_B)
irsens = infrared_sensor()

assert lmotor.connected, "Left motor is not connected!"
assert rmotor.connected, "Right motor is not connected!"
assert irsens.connected, "Infrared sensor is not connected!"

lmotor.speed_regulation_enabled = 'on'
rmotor.speed_regulation_enabled = 'on'

lmotor.stop_command = 'brake'
rmotor.stop_command = 'brake'

rc = remote_control(irsens)

def make_roll(m, p):
    def roll(state):
        if state:
            (m.speed_sp, m.command) = (p * 900, 'run-forever')
        else:
            m.command = 'stop'
    return roll

rc.on_red_up   (make_roll(lmotor,  1))
rc.on_red_down (make_roll(lmotor, -1))
rc.on_blue_up  (make_roll(rmotor,  1))
rc.on_blue_down(make_roll(rmotor, -1))

while True:
    rc.process()
    sleep(0.01)

