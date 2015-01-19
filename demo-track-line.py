#!/usr/bin/python

# Line tracking program
# The algorithm is taken from the following link:
# http://thetechnicgear.com/2014/03/howto-create-line-following-robot-using-mindstorms/
#
# The default values for Kp, Ki, and Kd (see the link above) are only
# guaranteed to work for my specific setup and most probably will need to be
# adjusted on a case-by-case basis.

import sys, argparse, time, atexit
from ev3dev import *

#----------------------------------------------------------------------------
# Parse command line
#----------------------------------------------------------------------------
parser = argparse.ArgumentParser(sys.argv[0])
parser.add_argument(
    '--Kp', dest='Kp', type=float, default=30
    )
parser.add_argument(
    '--Ki', dest='Ki', type=float, default=3
    )
parser.add_argument(
    '--Kd', dest='Kd', type=float, default=10
    )
args = parser.parse_args(sys.argv[1:])

#----------------------------------------------------------------------------
# Initialize motors and sensors
#----------------------------------------------------------------------------
lmotor = large_motor(OUTPUT_C)
rmotor = large_motor(OUTPUT_B)

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

cs = color_sensor()
check_sensor(cs, 'Color')

#----------------------------------------------------------------------------
# Calibrate the color sensor.
# Put the color sensor on white/black surface and
# press touch sensor to take the reading.
#----------------------------------------------------------------------------
cs.mode = color_sensor.mode_reflect

def get_reading(color):
    sound.speak("Show me %s!" % color, True)
    while not ts.value(): time.sleep(0.1)

    sound.speak("OK", True)
    v = cs.value()
    print("%s: %s" %(color, v))
    return v


white = get_reading('white')
black = get_reading('black')
mid   = 0.5 * (white + black)

#----------------------------------------------------------------------------
# Follow the line!
#----------------------------------------------------------------------------
def turn(value):
    base = -500

    lmotor.pulses_per_second_setpoint = base + value
    rmotor.pulses_per_second_setpoint = base - value


lmotor.regulation_mode = motor.mode_on
lmotor.run_mode = motor.run_mode_forever

rmotor.regulation_mode = motor.mode_on
rmotor.run_mode = motor.run_mode_forever

lmotor.run()
rmotor.run()

print("Kp: %s" % args.Kp)
print("Ki: %s" % args.Ki)
print("Kd: %s" % args.Kd)

last_error = 0
integral   = 0

while not ts.value():
    error = mid - cs.value()
    integral = 0.5 * integral + error
    derivative = error - last_error
    last_error = error

    correction = args.Kp * error + args.Ki * integral + args.Kd * derivative

    turn(int(correction))
    time.sleep(0.01)
