from ev3dev_ext import *
from version import __version__
from PIL import Image, ImageDraw

# Furnish mode_set class (which is wrapper around std::set<std::string>)
# with __repr__ and __str__ methods which are better than defaults.
def mode_set_repr(self):
    return list(self).__repr__()

def mode_set_str(self):
    return list(self).__str__()

mode_set.__repr__ = mode_set_repr
mode_set.__str__  = mode_set_str

# Helper function to compute power for left and right motors when steering
def steering(direction, power=100):
    """
    Computes how fast each motor in a pair should turn to achieve the
    specified steering.
    
    Input:
        direction [-100, 100]: -100 means turn left as fast as possible,
            means drive in a straight line, and 100 means turn right as fast as
            possible.
        power: the outmost motor (the one rotating faster) should receive this
            value of power.

    Output:
        a tuple of power values for a pair of motors.
    """

    pl = power
    pr = power
    s = (50 - abs(float(direction))) / 50

    if direction >= 0:
        pr *= s
    else:
        pl *= s

    return (int(pl), int(pr))

# Stop a motor on destruction
def stop_taho_motor(self):
    self.set_command('stop')

large_motor.__del__ = stop_taho_motor
medium_motor.__del__ = stop_taho_motor

def stop_dc_motor(self):
    self.command = 'coast'

dc_motor.__del__ = stop_dc_motor

def stop_servo_motor(self):
    self.command = 'float'

servo_motor.__del__ = stop_servo_motor

# Provide a convenience wrapper for ev3dev.lcd class
class LCD(lcd):
    """
    A convenience wrapper for ev3dev.lcd class.
    Provides drawing functions from python imaging library (PIL).
    """

    def __init__(self):
        super(LCD, self).__init__()

        def alignup(n, m):
            r = n % m
            if r == 0:
                return n
            else:
                return n - r + m


        nx = alignup(self.resolution_x, 32)
        ny = self.resolution_y

        self.img = Image.new("1", (nx, ny), "white")

    @property
    def shape(self):
        """
        Dimensions of the LCD screen.
        """
        return (self.resolution_x, self.resolution_y)

    @property
    def draw(self):
        """
        Returns a handle to PIL.ImageDraw.Draw class associated with LCD.

        Example:
        lcd.draw.rectangle((10,10,60,20), fill=True)
        """
        return ImageDraw.Draw(self.img)

    def clear(self):
        """
        Clears the LCD.
        """
        self.draw.rectangle(((0,0),(self.shape)), fill="white")

    def update(self):
        """
        Applies pending changes to LCD.
        Nothing will be drawn on the screen until this function is called.
        """
        self.frame_buffer[:] = self.img.tobytes("raw", "1;IR")

