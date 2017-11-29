# Simple demo of of the PCA9685 PWM servo/LED controller library.
# This will move channel 0 from min to max position repeatedly.
# Author: Tony DiCola
# License: Public Domain
from __future__ import division
import time

# Import the PCA9685 module.
import Adafruit_PCA9685


# Initialise the PCA9685 using the default address (0x40).
pwm = Adafruit_PCA9685.PCA9685(0x6f)

# Alternatively specify a different address and/or bus:
#pwm = Adafruit_PCA9685.PCA9685(address=0x41, busnum=2)

# Configure min and max servo pulse lengths
servo_min = 400  # Min pulse length out of 4096
servo_max = 2600  # Max pulse length out of 4096

## Marker servo
# down = 1500
# up = 2250

# Slide servo
# left = 800
# right = 1900

# Set servo position in microseconds
def set_servo_pulse(channel, pulse):
    pulse_length = 1000000    # 1,000,000 us per second
    pulse_length //= 60       # 60 Hz
    pulse_length //= 4096     # 12 bits of resolution
    pulse //= pulse_length
    servo.set_pwm(channel, 0, pulse)

# Set frequency to 60hz, good for servos.
pwm.set_pwm_freq(60)

print('Moving servo on channel 0, press Ctrl-C to quit...')
while True:
    # Move servo on channel O between extremes.
    #pwm.set_pwm(0, 0, servo_min)
    pulse = int(input("Enter pulse: "))
    set_servo_pulse(0,pulse)
