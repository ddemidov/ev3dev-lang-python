#!/usr/bin/python

from PIL import Image, ImageDraw
import ev3dev

lcd = ev3dev.lcd()

# Use PIL to draw a smiley face.
# Pad image size to be divisible by 32
def alignup(n, m):
    r = n % m
    if r == 0:
        return n

    return n - r + m;

nx = alignup(lcd.resolution_x(), 32)
ny = lcd.resolution_y()

im = Image.new("1", (nx, ny), "white")

draw = ImageDraw.Draw(im)

draw.ellipse(( 20, 20,  60, 60))
draw.ellipse((118, 20, 158, 60))
draw.arc((20, 80, 158, 100), 0, 180)

# Copy image data to LCD's frame buffer
fb = lcd.frame_buffer()
ib = im.tobytes("raw", "1;IR")

fb[:] = ib
