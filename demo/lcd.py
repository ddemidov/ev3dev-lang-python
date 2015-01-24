#!/usr/bin/python
#
# Draws a smiley face on the brick screen.

import time
from ev3dev_utils.lcd import LCD

lcd = LCD()

# lcd.draw returns a PIL.ImageDraw handle
lcd.draw.ellipse(( 20, 20,  60, 60))
lcd.draw.ellipse((118, 20, 158, 60))
lcd.draw.arc((20, 80, 158, 100), 0, 180)

# Update lcd display
lcd.update()

time.sleep(3)
