from PIL import Image, ImageDraw
import ev3dev

class LCD:
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
        return (self.dev.resolution_x(), self.dev.resolution_y())

    @property
    def draw(self):
        return ImageDraw.Draw(self.img)

    def clear(self):
        self.draw.rectangle(((0,0),(self.shape)), fill="white")

    def update(self):
        fb = self.dev.frame_buffer()
        fb[:] = self.img.tobytes("raw", "1;IR")


