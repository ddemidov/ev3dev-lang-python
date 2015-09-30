#!/usr/bin/python

print(
"""This demo make a robot driven by two large_motors follow a dark line with
help of a color sensor.

The algorithm is taken from the following link:
http://thetechnicgear.com/2014/03/howto-create-line-following-robot-using-mindstorms/

The default values for Kp, Ki, and Kd (see the link above) are only
guaranteed to work for my specific setup and most probably will need to be
adjusted on a case-by-case basis.

When started, the robot will ask to show him white (background) and dark (line)
colors. Place the robot so that the color sensor would be directly above the
respective color and press the touch sensor to proceed.

Press the touch sensor to stop the program.

Required hardware:
    large motor on output port B
    large motor on output port C
    color sensor on any input port
    touch sensor on any input port
"""
)

import sys, argparse, time
from ev3dev import *

#----------------------------------------------------------------------------
# Parse command line
#----------------------------------------------------------------------------
parser = argparse.ArgumentParser(sys.argv[0])
parser.add_argument('--Kp', dest='Kp', type=float, default=3.0)
parser.add_argument('--Ki', dest='Ki', type=float, default=0.5)
parser.add_argument('--Kd', dest='Kd', type=float, default=2.0)
args = parser.parse_args(sys.argv[1:])

#----------------------------------------------------------------------------
# Initialize motors and sensors
#----------------------------------------------------------------------------
lmotor = large_motor(OUTPUT_C); assert lmotor.connected
rmotor = large_motor(OUTPUT_B); assert rmotor.connected
cs     = color_sensor();        assert cs.connected
ts     = touch_sensor();        assert ts.connected

#----------------------------------------------------------------------------
# Calibrate the color sensor.
# Put the color sensor on white/black surface and
# press touch sensor to take the reading.
#----------------------------------------------------------------------------
cs.mode = 'COL-REFLECT'

def get_reading(color):
    sound.speak("Show me %s!" % color, True)
    while not ts.value(): time.sleep(0.1)

    v = cs.value()
    print("%s: %s" %(color, v))
    sound.speak("OK", True)
    return v


white = get_reading('white')
black = get_reading('black')
mid   = 0.5 * (white + black)

#----------------------------------------------------------------------------
# Follow the line!
#----------------------------------------------------------------------------
lmotor.speed_regulation_enabled = 'on'
rmotor.speed_regulation_enabled = 'on'

last_error = 0
integral   = 0

while not ts.value():
    error      = mid - cs.value()
    integral   = 0.5 * integral + error
    derivative = error - last_error
    last_error = error

    correction = args.Kp * error + args.Ki * integral + args.Kd * derivative

    for m,p in zip((lmotor, rmotor), steering(correction, 540)):
        m.run_forever(speed_sp=p)

    time.sleep(0.01)
