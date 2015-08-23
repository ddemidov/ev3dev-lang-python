#!/usr/bin/python

import sys, time
from ev3dev import *
# The mindstorms_widgets class is a wrapper for the python-ev3dev
# language binding.  It implements interfaces that correspond directly
# to the action and sensor programming blocks (widgets) from the 
# mindstorms LabVIEW software.  This allows leverage from the amazing
# documentation already provided by Lego.  Some exceptions are:
# * It doesn't auto-detect the devices.  You need to call
#   connect_motor or connect_sensor first for each device you use
# * Only two large motors and one medium motor are supported.
#   This simplifies the interfaces.  The motors are referenced by
#   the following names in the interfaces:
#   'medium' motor, default Port A
#   'right'  large motor, default Port B
#   'left'   large motor, default Port C
# * degrees and rotations inputs must always be positive
#   use negative power to go backward
# * Rather than having a separate unregulated_motor widget, regulated is an
#   an optional input parameter (default = True)
# * motor_rotation_measure, 'current_power' returns both the duty_cycle_sp
#   (the actual power level being applied to the motor) and speed_sp
# * sound includes an option to give a sentence and have the robot speak

# Known issues/limitations
# * The degrees, rotations, speed and invert values providied as inputs do
#   not map directly to the values programmed to the sysfs attributes i.e.
#   if you don't use mindstorm_widgets methods exclusively to control the 
#   motors you will need to explicitly initialize all the relevant sysfs
#   attributes each time you program the motor from outside mindstorms_widgets
# * led_flash
# * no error handling/recovery
# * volume for tones doesn't work very well (seems to have only 2 levels)
# * when reading the heading and proximity in beacon mode, ir_sensor_measure
#   returns good values at first but continuing to poll it seems to result
#   in inconsistent values
# * ultrasonic and gyro sensors are not implemented

# Mindstorms uses the following coefficients to compute speed_sp
# based on a power input in range 0-100 i.e. speed_sp = x * power
# medium motor: x = 16.4
# large motor : x = 10.3
# e.g. power of 10 sets medium motor speed_sp to 160 (deg/sec)

MEDIUM_MOTOR_SPEED_COEFFICIENT = 16.4
LARGE_MOTOR_SPEED_COEFFICIENT  = 10.3

class mindstorms_widgets:
    def __init__(self):
        self.motor = {
            'medium':None,
            'left'  :None,
            'right' :None
        }

    def _set_motor_attribs( self, size, motors, powers,
                            invert, regulated, brake_at_end ):
        # For regulated speed, speed_sp is tacho counts per 
        # second which for lego EV3 motors is also degrees per second.
        # To provide a mindstorms 0-100 power value for regulated
        # speed we need to convert it.
        speed_x = LARGE_MOTOR_SPEED_COEFFICIENT
        if size == 'medium':
            speed_x = MEDIUM_MOTOR_SPEED_COEFFICIENT
            
        for m,p in zip ( motors, powers ):
            # When using run_forever() the sign of speed_sp or duty_cycle_sp
            # determines the direction the motor runs.  When using
            # run_to_rel_pos() the sign of position_sp determines the
            # direction and the sign of speed_sp/duty_cycle_sp is ignored by
            # the driver.
            
            # In either case the polarity reverses the direction.
            
            # When using move_steering, providing a negative power should
            # result in reversing along the same path of a positive power
            # in the same direction.  If the steering is sharp the power of
            # one motor is opposite sign of the other.
            
            # This all combines to make for overly complicated logic when the
            # intent is to set motor attribs independent of knowing what 
            # methods are being used.  To avoid this complexity I devised the
            # following scheme:
            # * Use the motor polarity as the common attribute to determine
            #   direction since it works the same for run_forever() and 
            #   run_to_rel_pos()
            # * Always program duty_cycle_sp and speed_sp to positive values
            # * Always program position_sp to a positive value  
            pol = ['normal','inversed']
            brk = ['coast', 'brake']
            inv = invert
            if p < 0:
                inv = not invert
                p = abs( p )
            m.polarity = pol[int( inv )] #e.g. True=1='inversed'
            m.stop_command = brk[int( brake_at_end )]
            print ('speed: '+str(p)+' inv: '+str(inv))
            if regulated:
                m.speed_regulation_enabled = 'on'
                m.speed_sp = int( p*speed_x )
            else:
                m.speed_regulation_enabled = 'off'
                m.duty_cycle_sp = p
                
    def _move_wait( self ):
        # wait for any running motors to finish their movement then stop
        # all the motors.
        left_is_off = False
        right_is_off = False
        if 'running' not in self.motor['left'].state:
            left_is_off = True
        if 'running' not in self.motor['right'].state:
            right_is_off = True
        while ((left_is_off or 'running' in self.motor['left'].state) and
               (right_is_off or 'running' in self.motor['right'].state)):
            time.sleep(0.1)
        self.motor['left'].stop()
        self.motor['right'].stop()
        
    def _move( self, mode, seconds, degrees, rotations ):
        if mode == 'off':
            self.motor['left'].stop()
            self.motor['right'].stop()
        elif mode == 'on':
            self.motor['left'].run_forever()
            self.motor['right'].run_forever()
        elif mode == 'on_for_seconds':
            self.motor['left'].run_forever()
            self.motor['right'].run_forever()
            time.sleep(seconds)
            self.motor['left'].stop()
            self.motor['right'].stop()
        elif mode == 'on_for_degrees':
            self.motor['left'].run_to_rel_pos(position_sp=degrees)
            self.motor['right'].run_to_rel_pos(position_sp=degrees)
            self._move_wait()
        elif mode == 'on_for_rotations':
            self.motor['left'].run_to_rel_pos( position_sp=int(rotations*360) )
            self.motor['right'].run_to_rel_pos( position_sp=int(rotations*360) )
            self._move_wait()
            
    def _validate_inputs( self, degrees, rotations ):
        if (degrees < 0 or rotations < 0):
            print( 'degrees and rotations should be positive. '
                   'Use negative power to go backward' )

    def _run_motor( self, motor, mode, power, 
                    seconds, degrees, rotations,
                    invert, regulated, brake_at_end ):
        self._validate_inputs( degrees, rotations )
        self._set_motor_attribs( 'medium', [motor], [power], invert, regulated,
                                 brake_at_end )
        if mode == 'off':
            motor.stop()
        elif mode == 'on':
            motor.run_forever()
        elif mode == 'on_for_seconds':
            motor.run_forever()
            time.sleep(seconds)
            motor.stop()
        elif mode == 'on_for_degrees':
            motor.run_to_rel_pos(position_sp=degrees)
            while 'running' in motor.state:
                time.sleep(0.1)
        elif mode == 'on_for_rotations':
            motor.run_to_rel_pos(position_sp=int(rotations*360))
            while 'running' in motor.state:
                time.sleep(0.1)
                
    def _led_pulse( self, color ):
        # Note that calling flash() multiple times on a LED causes an error
        # Disable this routine for now
        return
    
        if color == 'green':
            led.green_left.flash( 200, 200 )
            led.green_right.flash( 200, 200 )
        elif color == 'orange':
            led.green_left.flash( 200, 200 )
            led.red_left.flash( 200, 200 )
            led.green_right.flash( 200, 200 )
            led.red_right.flash( 200, 200 )
        elif color == 'red':
            led.red_left.flash( 200, 200 )
            led.red_right.flash( 200, 200 )
        

    # Begin public methods
        
    def connect_motor( self, motor, port=None ):
        default_port = {
            'medium':'A',
            'left'  :'B',
            'right' :'C'
        }

        port_enum = {
            'A' :OUTPUT_A,
            'B' :OUTPUT_B,
            'C' :OUTPUT_C,
            'D' :OUTPUT_D
        }
        if not port:
            port = default_port[motor]
        port = port.upper()
            
        if motor == 'medium':
            self.motor['medium'] = medium_motor(port_enum[port])
            if not self.motor['medium'].connected:
                print("Medium motor not connected to port " + port)
        elif motor == 'right':
            self.motor['right'] = large_motor(port_enum[port])
            if not self.motor['right'].connected:
                print("Large right motor not connected to port " + port)
        elif motor == 'left':
            self.motor['left'] = large_motor(port_enum[port])
            if not self.motor['left'].connected:
                print("Large left motor not connected to port " + port)
            
    def connect_sensor( self, sensor ):
        if sensor == 'color':
            self.cs = color_sensor()
            if not self.cs.connected:
                print("Color sensor not connected")
        elif sensor == 'touch':
            self.ts = touch_sensor()
            if not self.ts.connected:
                print("Touch sensor not connected")
        elif sensor == 'infrared':
            self.irs = infrared_sensor()
            if not self.irs.connected:
                print("Infrared sensor not connected")
                
    
    def large_motor( self, motor, mode, power=50, 
                     seconds=0, degrees=0, rotations=0,
                     invert=False, regulated=True, brake_at_end=True ):
        self._run_motor( self.motor[motor], mode, power, 
                         seconds, degrees, rotations,
                         invert, regulated, brake_at_end )

    def medium_motor( self, mode, power=50,  
                      seconds=0, degrees=0, rotations=0,
                      invert=False, regulated=True, brake_at_end=True ):
        self._run_motor( self.motor['medium'], mode, power, 
                         seconds, degrees, rotations, 
                         invert, regulated, brake_at_end )

    def move_steering( self, mode, direction=0, power=50, 
                       seconds=0, degrees=0, rotations=0,
                       invert=False, regulated=True, brake_at_end=True ):
        '''
        direction [-100, 100]:
        * -100 = pivot left i.e. right motor = power
        *                        left motor  = -power
        * -50  = sharp left i.e. right motor = power
        *                        left motor  = 0
        * -25  = turn  left i.e. right motor = power
        *                        left motor  = 1/2 power
        * 0    = straight
        * 25   = turn right
        * 50   = sharp right
        * 100  = pivot right
        '''
        self._validate_inputs( degrees, rotations )
        powers = steering( direction, power )
        self._set_motor_attribs( 'large', 
                                 [self.motor['left'], self.motor['right']], 
                                 powers, invert, regulated, brake_at_end )
        self._move( mode, seconds, degrees, rotations )
        
    def move_tank( self, mode, direction=0, lr_power=None,
                   seconds=0, degrees=0, rotations=0,
                   invert=False, regulated=True, brake_at_end=True ):
        self._validate_inputs( degrees, rotations )
        if not lr_power:
            lr_power=[50,50]
        self._set_motor_attribs( 'large',
                                 [self.motor['left'], self.motor['right']],
                                 lr_power, invert, regulated, brake_at_end )
        self._move( mode, seconds, degrees, rotations )

    def brick_status_light( self, mode, color='green', pulse=False ):
        led.all_off()
        if mode == 'on':
            if color == 'green':
                led.green_on()
            elif color == 'orange':
                led.all_on()
            elif color == 'red':
                led.red_on()
        if mode == 'on' and pulse:
            self._led_pulse( color )
            pass
            
    def color_sensor_measure( self, mode ):
        if mode == 'color':
            self.cs.mode = 'COL-COLOR'
        elif mode == 'reflected_light_intensity':
            self.cs.mode = 'COL-REFLECT'
        elif mode == 'ambient_light_intensity':
            self.cs.mode = 'COL-AMBIENT'
        return self.cs.value()

    def ir_sensor_measure( self, mode, channel=1 ):
        # value(n) returns the given value. See 
        # http://www.ev3dev.org/docs/sensors/lego-ev3-infrared-sensor/
        i = 2*(channel-1)
        if mode == 'proximity':
            self.irs.mode = 'IR-PROX'
            return self.irs.value()
        elif mode == 'beacon':
            self.irs.mode = 'IR-SEEK'
            # (heading, proximity, detected)
            # heading in range (-25,25)
            # proximity is -128 or in [0,100]
            prox = self.irs.value(i+1)
            return( self.irs.value(i), prox, prox != -128 )
        elif mode == 'remote':
            self.irs.mode = 'IR-REMOTE'
            '''
            0 = No button (and Beacon Mode is off)
            1 = Button 1
            2 = Button 2
            3 = Button 3
            4 = Button 4
            5 = Both Button 1 and Button 3
            6 = Both Button 1 and Button 4
            7 = Both Button 2 and Button 3
            8 = Both Button 2 and Button 4
            9 = Beacon Mode is on
            10 = Both Button 1 and Button 2
            11 = Both Button 3 and Button 4
            '''
            return self.irs.value( channel-1 )
        

    def motor_rotation_measure( self, motor, mode ):
        motor = self.motor[motor]
        if mode == 'reset':
            motor.position = 0
            return None
        if mode == 'current_power':
            return (motor.speed, motor.duty_cycle)
        degrees=motor.position
        if mode == 'degrees':
            return degrees
        elif mode == 'rotations':
            return degrees/360
            
    def touch_sensor_measure( self ):
        return self.ts.value()

    def sound( self, mode, volume=40, hz=523, path=None, 
               sentence=None, duration_ms=200 ):
        # todo:Volume doesn't seem to work very well for tones.  
        #       0=just a click
        #       1-49 same loud volume tone
        #       > 50 same volume really loud tone
        sound.volume = volume
        if mode == 'play_file':
            # Sound.rsf files can be extracted from mindstorms by using the
            # sound in a mindstorms program then going to mindstorms 
            # project->sounds tab and choosing "export"

            sound.play( path )
        elif mode == 'play_tone':
            sound.tone( int(hz), duration_ms )
        elif mode == 'speak':
            sound.speak( sentence, True )
            
    