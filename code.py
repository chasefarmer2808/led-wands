import time
import math
import board
import busio
import adafruit_lis3dh
import neopixel
from digitalio import DigitalInOut, Direction, Pull

i2c = board.I2C()  # uses board.SCL and board.SDA
lis3dh = adafruit_lis3dh.LIS3DH_I2C(i2c)

pixel_pin = board.D4
num_pixels = 4

RED = (255, 0, 0, 0)
BLACK = (0, 0, 0, 0)

pixels = neopixel.NeoPixel(pixel_pin, num_pixels, brightness=0.3, auto_write=False, pixel_order=(1, 0, 2, 3))

# Set range of accelerometer (can be RANGE_2_G, RANGE_4_G, RANGE_8_G or RANGE_16_G).
lis3dh.range = adafruit_lis3dh.RANGE_4_G

sample_btn = DigitalInOut(board.D3)
sample_btn.direction = Direction.INPUT
sample_btn.pull = Pull.UP

max_accel = 0
reset_max = False

def color_chase(color, wait):
    for i in range(num_pixels):
        pixels[i] = color
        time.sleep(wait)
        pixels.show()
    time.sleep(0.5)

def sample_accel():
    # Read accelerometer values (in m / s ^ 2).  Returns a 3-tuple of x, y,
    # z axis values.  Divide them by 9.806 to convert to Gs.
    x, y, z = [
        value / adafruit_lis3dh.STANDARD_GRAVITY for value in lis3dh.acceleration
    ]

    return math.sqrt(x*x + y*y + z*z)

# Loop forever printing accelerometer values 
while True:
    pixels.fill(BLACK)
    if not sample_btn.value:
        if reset_max:
            max_accel = 0
            reset_max = False

        total_accel = sample_accel()

        if total_accel > max_accel:
            max_accel = int(total_accel)

        print(total_accel)
        # # Small delay to keep things responsive but give time for interrupt processing.
        time.sleep(0.1)
    else:
        curr_max_accel = max_accel
        reset_max = True

        if curr_max_accel is 0:
            pixels.fill(BLACK)
            pixels.show()

        print(curr_max_accel)

        for i in range(0, curr_max_accel):
            pixels[i] = RED
            pixels.show()