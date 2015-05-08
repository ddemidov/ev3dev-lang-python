#!/usr/bin/python

print(
"""This demo allows to control motors attached to each of the output ports with the
brick buttons. This could be very helpful for initial tests of your robot's
functionality.

Use left/right buttons to choose output port to control.
Use up/down buttons to run the motor attached to the current port.
"""
)

from time import sleep
from ev3dev import *

lcd = LCD()
motors = [
        motor(OUTPUT_A),
        motor(OUTPUT_B),
        motor(OUTPUT_C),
        motor(OUTPUT_D)
        ]

port = 0

def draw():
    nx = lcd.shape[0]
    ny = lcd.shape[1]

    lcd.clear()

    dx = nx / 4
    lcd.draw.line((0, 32, nx, 32), width=2)
    for i in [0, dx, 2 * dx, 3 * dx, nx-2]:
        lcd.draw.line((i, 0, i, 32), width=2)

    pos = [dx/2, dx/2 + dx, dx/2 + 2 * dx, dx / 2 + 3 * dx]
    for p in zip(pos, ['A', 'B', 'C', 'D']):
        lcd.draw.text((p[0], 16), p[1])

    x = pos[port] + 8
    lcd.draw.rectangle((x, 4, x + 4, 8), fill='black')

    lcd.update()

def drive(m = None, state=False, power=None):
    if not m.connected: return

    if state:
        m.run_forever(speed_regulation_enabled='off', duty_cycle_sp=power)
    else:
        m.stop()

while not button.back.pressed:
    if button.left.pressed:
        port = (port - 1) % 4
    elif button.right.pressed:
        port = (port + 1) % 4

    if button.up.pressed:
        drive(motors[port], True,  100)
    elif button.down.pressed:
        drive(motors[port], True, -100)
    else:
        drive(motors[port], False)

    draw()

    sleep(0.1)
