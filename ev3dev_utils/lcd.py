from PIL import Image, ImageDraw
import ev3dev

class LCD:
    """
    A convenience wrapper around ev3dev.lcd class.
    Provides drawing functions from python imaging library (PIL).
    """

    def __init__(self):
        self.dev = ev3dev.lcd()

        def alignup(n, m):
            r = n % m
            if r == 0:
                return n
            else:
                return n - r + m


        self.nx = alignup(self.dev.resolution_x(), 32)
        self.ny = self.dev.resolution_y()

        self.img = Image.new("1", (self.nx, self.ny), "white")

    @property
    def shape(self):
        """
        Dimensions of the LCD screen.
        """
        return (self.dev.resolution_x(), self.dev.resolution_y())

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
        fb = self.dev.frame_buffer()
        fb[:] = self.img.tobytes("raw", "1;IR")


