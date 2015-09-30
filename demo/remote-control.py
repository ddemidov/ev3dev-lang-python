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

lmotor = large_motor(OUTPUT_C); assert lmotor.connected
rmotor = large_motor(OUTPUT_B); assert rmotor.connected
irsens = infrared_sensor();     assert irsens.connected

rc = remote_control(irsens)

lmotor.set(speed_regulation_enabled='on', stop_command='brake')
rmotor.set(speed_regulation_enabled='on', stop_command='brake')

def make_roll(m, p):
    def roll(state):
        if state:
            m.run_forever(speed_sp=p*900)
        else:
            m.stop()
    return roll

rc.on_red_up   (make_roll(lmotor,  1))
rc.on_red_down (make_roll(lmotor, -1))
rc.on_blue_up  (make_roll(rmotor,  1))
rc.on_blue_down(make_roll(rmotor, -1))

while True:
    rc.process()
    sleep(0.01)

