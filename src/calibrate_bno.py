from Adafruit_BNO055 import BNO055
import time
import sys
import os

'''
Obtain Calibrations Values for BNO sensor
This needs to be done anytime the mechanical configuration of the robot changes
It will run the calibration until all system statuses equal 3
Gyro Cal(easy): Place robot on flat surface and let it rest. Make sure the robot stays super stationary
Mag Cal(easy): Rotate robot on all 3 axis
Accel Cal(easy): Place robot on all six sides for 10 seconds each. Repeat until robot calibration is complete
'''


# Figure out serial device ports
cmd = "sudo udevadm info --query=property --name=/dev/ttyUSB0 | grep SERIAL="
os.system(cmd + " > tmp")
result = open('tmp', 'r').read()
os.remove('tmp')
if "FTDI" in result:
    bno_port = "/dev/ttyUSB1"
else:
    bno_port = "/dev/ttyUSB0"


# Connect to IMU
bno = BNO055.BNO055(serial_port=bno_port)
if not bno.begin():
    raise RuntimeError('Failed to initialize BNO055! Is the sensor connected?')

while True:

    sys,gyro,accel,mag = bno.get_calibration_status()
    print("sys: {}, gyro: {}, acc: {}, mag: {}".format(sys,gyro,accel,mag))

    if sys == 3 and gyro == 3 and accel == 3 and mag == 3:
        print(bno.get_calibration())
        break
