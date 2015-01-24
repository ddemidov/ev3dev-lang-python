#!/usr/bin/python
#
# Line tracking program
# The algorithm is taken from the following link:
# http://thetechnicgear.com/2014/03/howto-create-line-following-robot-using-mindstorms/
#
# The default values for Kp, Ki, and Kd (see the link above) are only
# guaranteed to work for my specific setup and most probably will need to be
# adjusted on a case-by-case basis.

import sys, argparse, time
from ev3dev import *
from ev3dev_utils.motors import *

#----------------------------------------------------------------------------
# Parse command line
#----------------------------------------------------------------------------
parser = argparse.ArgumentParser(sys.argv[0])
parser.add_argument('--Kp', dest='Kp', type=float, default=3)
parser.add_argument('--Ki', dest='Ki', type=float, default=0.5)
parser.add_argument('--Kd', dest='Kd', type=float, default=2)
args = parser.parse_args(sys.argv[1:])

#----------------------------------------------------------------------------
# Initialize motors and sensors
#----------------------------------------------------------------------------
lmotor = large_motor(OUTPUT_C)
rmotor = large_motor(OUTPUT_B)

ts = touch_sensor()
cs = color_sensor()

#----------------------------------------------------------------------------
# Calibrate the color sensor.
# Put the color sensor on white/black surface and
# press touch sensor to take the reading.
#----------------------------------------------------------------------------
cs.mode = color_sensor.mode_reflect

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
lmotor.regulation_mode = motor.mode_on
rmotor.regulation_mode = motor.mode_on

last_error = 0
integral   = 0

while not ts.value():
    error = mid - cs.value()
    integral = 0.5 * integral + error
    derivative = error - last_error
    last_error = error

    correction = args.Kp * error + args.Ki * integral + args.Kd * derivative

    drive_for(ever=True, left=lmotor, right=rmotor, dir=correction, power=60)
    time.sleep(0.01)
