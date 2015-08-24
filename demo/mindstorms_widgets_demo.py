#!/usr/bin/python

print(
"""
This demo exercises the mindstorms_widgets as well as use of the
brick display and buttons.
Run this script from the Brickman File Browser.
* (green) action widgets from the mindstorms (LabVIEW) software
  - medium_motor, large_motor, move_steering, move_tank, status_light, sound
* (yellow) sensor widgets
  - color_sensor, infrared_sensor, motor_rotation, touch_sensor
"""
)

import sys, time, os
from ev3dev import *
from mindstorms_widgets import mindstorms_widgets

color = ['none','black','blue','green','yellow','red','white','brown']

robot = mindstorms_widgets()
lcd    = LCD()
(hpix,ypix) = lcd.shape
selection = 0
lcd.y_pos = 0 
#Current y coordinate of cursor

def wait_for_back_button():
    brick_print( '<back> to main menu' )
    while not button.back.pressed:
        time.sleep(0.1)
    robot.sound( 'play_tone', hz=700, duration_ms=100 )
    
def wait_for_enter():
    while not button.enter.pressed:
        time.sleep(0.1)
        
def demo_medium_motor():
    draw_header( 'Medium Motor' )
    wait_for_enter()
    robot.medium_motor( 'on', power=100, regulated=False )
    brick_print( 'run at full speed, coast stop' )
    time.sleep(2)
    robot.medium_motor( 'off', brake_at_end=False )
    time.sleep(2)
    brick_print( 'On for 2.5s, regulated speed' )
    robot.medium_motor( 'on_for_seconds', power=60, seconds=2.5 )
    wait_for_back_button()

def demo_large_motor():
    draw_header( 'Large Motor' )
    brick_print( '<enter> to start' )
    wait_for_enter()
    brick_print( 'right large motor goes fwd' )
    robot.large_motor( 'right', 'on_for_rotations', rotations=2.5 )
    time.sleep(1)
    brick_print( 'left large motor goes back' )
    robot.large_motor( 'left', 'on_for_degrees', degrees=700, power=-60 )
    wait_for_back_button()

def demo_move_steering():
    draw_header( 'Move Steering' )
    brick_print( '<enter> to start' )
    wait_for_enter()
    brick_print( 'fwd and back, coast to stop' )
    robot.move_steering( 'on', power=100 )
    time.sleep(1)
    robot.move_steering( 'on', power=-100 )
    time.sleep(1)
    robot.move_steering( 'off', brake_at_end=False )
    time.sleep(2)
    brick_print( 'figure 8' )
    fct = 8.5 # full circle time
    rad = 40 # Direction to make a good radius for the circle
    robot.move_steering( 'on_for_seconds', direction=rad, power=70,
                         seconds=fct/2 )
    robot.move_steering( 'on_for_seconds', direction=-rad, power=70,
                         seconds=fct )
    robot.move_steering( 'on_for_seconds', direction=rad, power=70,
                         seconds=fct/2 )
    time.sleep(5)
    brick_print( 'three point turn' )
    robot.move_steering( 'on_for_degrees', direction=50, degrees=1000 )
    robot.move_steering( 'on_for_rotations', direction=-50, power=-60, 
                         rotations=3, brake_at_end=False )
    robot.move_steering( 'on_for_seconds', power=70, seconds=1 )
    robot.sound( 'play_file', path='~/dog_bark.rsf' )
    wait_for_back_button()

def demo_move_tank():
    draw_header( 'Move Tank' )
    brick_print( '<enter> to start' )
    wait_for_enter()
    brick_print( 'Left then right' )
    robot.move_tank( 'on_for_seconds', lr_power=[10,50], seconds=2.5, 
                     regulated=False )
    robot.move_tank( 'on_for_seconds', lr_power=[50,10], seconds=2.5 )
    wait_for_back_button()

def demo_status_light():
    draw_header( 'Status Light' )
    brick_print( 'Off' )
    robot.brick_status_light( 'off' )
    time.sleep(3)
    brick_print( 'Solid red' )
    robot.brick_status_light( 'on', color='red' )
    time.sleep(3)
    brick_print( 'Flashing orange' )
    robot.brick_status_light( 'on', color='orange', pulse=True )
    time.sleep(3)
    brick_print( 'Solid orange' )
    robot.brick_status_light( 'on', color='orange' )
    time.sleep(3)
    brick_print( 'Back to green' )
    robot.brick_status_light( 'on', color='green' )
    wait_for_back_button()

def demo_color_sensor():
    draw_header( 'Color Sensor' )
    robot.sound( 'speak', sentence='At each prompt press the touch sensor '
                 'to continue' )
    brick_print( 'Detect color, press <ts>' )
    while not robot.touch_sensor_measure(): time.sleep(0.1)
    val = robot.color_sensor_measure( mode='color' )
    brick_print( 'Detected color = '+color[val] )
    robot.sound( 'speak', sentence='That looks like the color ' + color[val] )
    brick_print( 'Detect ambient light, press <ts>' )
    while not robot.touch_sensor_measure(): time.sleep(0.1)
    val = robot.color_sensor_measure( mode='ambient_light_intensity' )
    brick_print( 'Detected val = '+str(val) )
    robot.sound( 'speak', sentence='Ambient light level is ' + str(val) )
    brick_print( 'Detect reflected light, press <ts>' )
    while not robot.touch_sensor_measure(): time.sleep(0.1)
    val = robot.color_sensor_measure( mode='reflected_light_intensity' )
    brick_print( 'Detected val = '+str(val) )
    robot.sound( 'speak', sentence='Reflected light level is ' + str(val) )
    wait_for_back_button()
    
def demo_ir_sensor():
    draw_header( 'IR Sensor' )
    brick_print( 'Proximity - <enter> to stop' )
    while not button.enter.pressed:
        val=robot.ir_sensor_measure('proximity')
        print( 'proximity = '+str(val) )
        time.sleep(1)

    time.sleep(5)
    brick_print( 'Beacon ch2 - <enter> to stop' )
    while not button.enter.pressed:
        (heading, proximity, detected) = robot.ir_sensor_measure( 'beacon', 
                                                                  channel=2 )
        print( 'beacon channel 2 (heading=%s, proximity=%s, detected=%s)' % (
                str(heading), str(proximity), str(detected)) )
        time.sleep(1)

    time.sleep(5)
    brick_print( 'Remote ch1 - <enter> to stop' )
    while not button.enter.pressed:
        val = robot.ir_sensor_measure( 'remote' )
        print( 'remote channel 1 key = ' + str(val) )
        time.sleep(1)
    wait_for_back_button()

def run_med_motor( regulate ):
    robot.motor_rotation_measure( 'medium', 'reset' )
    for s in range (10,101,10):
        robot.medium_motor( 'on', power=s, regulated=regulate )
        time.sleep(1)
        rotations=robot.motor_rotation_measure( 'medium', 'rotations' )
        degrees=robot.motor_rotation_measure( 'medium', 'degrees' )
        (speed_sp,duty_cycle_sp) = \
            robot.motor_rotation_measure( 'medium', 'current_power' )
        # speed_sp is in degrees/sec, duty_cycle_sp is total power
        print( 'power=%s, duty_cycle=%s, deg/sec=%s, rotations=%s, '
               'degrees=%s' %( str(s), str(duty_cycle_sp), str(speed_sp),
                               str(rotations), str(degrees)) )
    robot.medium_motor( 'off' )
    
def demo_motor_rotation():
    draw_header( 'Motor Rotation' )
    brick_print( '<enter> to start' )
    wait_for_enter()
    brick_print( 'Medium motor, regulated speed' )
    print( 'Medium motor, regulated speed' )
    run_med_motor( True )
    brick_print( 'Medium motor, unregulated' )
    print( 'Medium motor, unregulated speed' )
    run_med_motor( False )
    wait_for_back_button()

def draw_header( header ):
    lcd.clear()
    lcd.draw.text( (2,20), header )
    lcd.draw.line( (0, 32, hpix, 32), width=2 )
    lcd.y_pos = 40
    
def brick_print( line, fill='black', update=True ):
    lcd.draw.text( (2,lcd.y_pos), line, fill=fill )
    lcd.y_pos += 8
    if update:
        lcd.update()

def draw_menu( selection ):
    draw_header( 'Mindstorm Widgets (Back=exit)' )
    for i in range( 0, len(menu) ):
        if i == selection:
            lcd.draw.rectangle( [0, lcd.y_pos, hpix, lcd.y_pos+9], 
                                fill="black" )
            brick_print( '>' + menu[selection][1], fill='white',
                            update=False )
        else:
            brick_print( ' ' + menu[i][1], update=False )
    lcd.update()
            
menu = [
    [demo_medium_motor,    'Medium Motor'],
    [demo_large_motor,     'Large Motor'],
    [demo_move_steering,   'Move Steering'],
    [demo_move_tank,       'Move Tank'],
    [demo_status_light,    'Status Light'],
    [demo_color_sensor,    'Color Sensor'],
    [demo_ir_sensor,       'IR Sensor'],
    [demo_motor_rotation,  'Motor Rotation']
]

# Start script

robot.connect_motor( 'medium', port='D' )
robot.connect_motor( 'left' )
robot.connect_motor( 'right' )
robot.connect_sensor( 'touch' )
robot.connect_sensor( 'color' )
robot.connect_sensor( 'infrared' )

# Brickman uses virtual terminal 1 to control the brick display.
# If running the script from ssh you need to change the virtual terminal
# to avoid conflicting with Brickman
# os.system("sudo chvt 7")

while not button.back.pressed:
    if button.up.pressed:
        selection = (selection - 1) % len( menu )
        robot.sound( 'play_tone', volume=0 )
    elif button.down.pressed:
        selection = (selection + 1) % len( menu )
        robot.sound( 'play_tone', volume=0 )
    elif button.enter.pressed:
        # run the chosen demo
        draw_menu( selection )
        menu[selection][0]()
    draw_menu( selection )
    time.sleep(0.05)
    
# os.system("sudo chvt 1")