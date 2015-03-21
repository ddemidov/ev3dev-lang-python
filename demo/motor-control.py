#!/usr/bin/python
#
# A script that controls motors attached to each of the output ports with the
# brick buttons. This could be very helpful for initial tests of your robot's
# functionality.

import time
from PIL import Image, ImageDraw, ImageFont
from ev3dev import *
from ev3dev_utils.lcd import *
from ev3dev_utils.motors import *

lcd = LCD()
motors = [
        motor(OUTPUT_A),
        motor(OUTPUT_B),
        motor(OUTPUT_C),
        motor(OUTPUT_D)
        ]

current = 0

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

    x = pos[current] + 8
    lcd.draw.rectangle((x, 4, x + 4, 8), fill='black')

    lcd.update()

def drive(state, power=None):
    m = motors[current]

    if not m.connected: return

    if state:
        run_for(ever=True, motor=m, power=power)
    else:
        m.reset()

def next_port():
    global current
    drive(False)
    current = (current + 1) % 4

def prev_port():
    global current
    drive(False)
    current = (current - 1) % 4

while not button.back.pressed:
    if button.left.pressed:
        prev_port()
    elif button.right.pressed:
        next_port()

    if button.up.pressed:
        drive(True, 80)
    elif button.down.pressed:
        drive(True, -80)
    else:
        drive(False)

    draw()
    time.sleep(0.1)
