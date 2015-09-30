#!/usr/bin/python

print(
"""This demo creates a thereminvox (http://en.wikipedia.org/wiki/Theremin) out
of your ev3 with an infrared sensor attached.

Continuously sounds a tone of varying frequency whenever the infrared sensor
detects an object infront of it. The closer the object is, the lower the tone
frequency.

Required hardware:
    infrared sensor on any input port
"""
)

from time import sleep
from ev3dev import *

ir = infrared_sensor(); assert ir.connected

ir.mode = 'IR-PROX'

sound.beep()

min_freq = 100
max_freq = 1500

while True:
    v = float(ir.value())
    if v < 100:
        f = (v / 100) * (max_freq - min_freq) + min_freq
        sound.tone(int(f), 50)
        sleep(0.045)
