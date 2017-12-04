from open_interface import Roomba
from pos_filter import PosFilter
from pos_controller import PosController
from hedgehog import Hedgehog
from Adafruit_BNO055 import BNO055
import Adafruit_PCA9685
import time
import numpy as np
import signal
import sys
import math
import os

# Set servo position in microseconds
def set_servo_pulse(channel, pulse):
    pulse_length = 1000000    # 1,000,000 us per second
    pulse_length //= 60       # 60 Hz
    pulse_length //= 4096     # 12 bits of resolution
    pulse //= pulse_length
    servos.set_pwm(channel, 0, pulse)

def offset_waypoint(x,y,heading):
    marker_offset = 0.23495 # marker displacement in meters off the back of the roomba
    wx = x + marker_offset*math.cos(heading)
    wy = y + marker_offset*math.sin(heading)
    return wx,wy

# Figure out serial device ports
cmd = "sudo udevadm info --query=property --name=/dev/ttyUSB0 | grep SERIAL="
os.system(cmd + " > tmp")
result = open('tmp', 'r').read()
os.remove('tmp')
if "FTDI" in result:
    roomba_port = "/dev/ttyUSB0"
    bno_port = "/dev/ttyUSB1"
else:
    roomba_port = "/dev/ttyUSB1"
    bno_port = "/dev/ttyUSB0"


# Connect to Roomba
robot = Roomba(roomba_port)

# Connect to Hedgehog
hh = Hedgehog("/dev/ttyACM0")

# Connect servos
servos = Adafruit_PCA9685.PCA9685(0x6f)
marker = 0 # channel 0
slide = 1 # channel 1
## Marker servo
marker_down = 1500 #1900
marker_up = 2250
## Slide servo
slide_left = 1130
slide_right = 1900
# Default position
set_servo_pulse(marker, marker_up)
set_servo_pulse(slide, slide_right)

# Connect to IMU
bno = BNO055.BNO055(serial_port=bno_port)
if not bno.begin():
    raise RuntimeError('Failed to initialize BNO055! Is the sensor connected?')

# Set BNO calibration(run calibrate bno to get these values)
bno_cal = [254, 255, 24, 0, 19, 0, 5, 250, 86, 0, 249, 247, 255, 255, 255, 255, 0, 0, 232, 3, 14, 3]
bno.set_calibration(bno_cal)


# Create Position Filter
pos_filter = PosFilter(robot,hh,bno,encoder_xy_weight=1,encoder_heading_weight=0.0)

# Create Position Controller
pos_ctrl = PosController()


length = 6 # meters
width = 1 # meters
waypoints = np.array([[0, length, -90],
                        [0, 0, 90]
                     ])
'''
waypoints = np.array([[0, length, 90],
                     [width, length, 0],
                     [width,  0,  -90],
                     [0,  0,  180]])
'''
update_rate = 100 #hz
last_update = 0
cnt = 0
state = "transit"

pos_filter.align()


pnt = waypoints[0]
wx,wy,wheading = pnt[0],pnt[1],math.radians(pnt[2])
wx,wy = offset_waypoint(wx,wy,wheading)
pos_ctrl.set(wx,wy,wheading)

try:
    while True:
        if state == "transit":
            if time.time() - last_update > 1.0/update_rate:
                x,y,heading = pos_filter.update()
                bec_x,bec_y,bec_heading = pos_filter.get_beacon_est()
                odo_x, odo_y,odo_heading = pos_filter.get_odometry_est()
                print("Filter - X: {}m, Y: {}m, heading: {}deg".format(round(x,3),round(y,3),round(math.degrees(heading),3)))
                print("Beacon - X: {}m, Y: {}m, heading: {}deg".format(round(bec_x,3),round(bec_y,3),round(math.degrees(bec_heading),3)))
                print("Odometry X: {}m, Y: {}m, heading: {}deg".format(round(odo_x,3),round(odo_y,3),round(math.degrees(odo_heading),3)))
                print("------------------------------------------")
                speed, rad, arrived = pos_ctrl.update(x,y,heading)
                robot.drive(speed,rad)
                if arrived:
                    cnt += 1
                    pnt = waypoints[cnt%len(waypoints)]
                    wx,wy,wheading = pnt[0],pnt[1],math.radians(pnt[2])
                    wx,wy = offset_waypoint(wx,wy,wheading)
                    pos_ctrl.set(wx,wy,wheading)
                    state = "mark"
                    time.sleep(0.75)
                last_update = time.time()
            else:
                time.sleep(0.3/update_rate)

        if state == "mark":
            '''
            ### Draw point
            # slide marker center
            set_servo_pulse(slide, 1450)
            time.sleep(0.3)
            # put marker down
            set_servo_pulse(marker, marker_down)
            time.sleep(0.3)
            # Lift marker up
            set_servo_pulse(marker, marker_up)
            time.sleep(0.3)

            '''

            ### Draw a U shape

            # put marker down
            set_servo_pulse(marker, marker_down)
            time.sleep(0.3)

            # drive in reverse
            sx,sy,heading = pos_filter.update()
            robot.drive(-0.05,0)
            while True:
                cx,cy,heading = pos_filter.update()
                dist = math.sqrt((sx-cx)**2 + (sy-cy)**2)
                if dist > 0.03:
                    break
                time.sleep(0.3/update_rate)
            robot.drive(0,0)

            # slide marker left
            set_servo_pulse(slide, slide_left)
            time.sleep(1.5)

            # drive forward
            sx,sy,heading = pos_filter.update()
            robot.drive(0.05,0)
            while True:
                cx,cy,heading = pos_filter.update()
                dist = math.sqrt((sx-cx)**2 + (sy-cy)**2)
                if dist > 0.03:
                    break
                time.sleep(0.3/update_rate)
            robot.drive(0,0)

            # lift marker
            set_servo_pulse(marker, marker_up)
            time.sleep(0.3)

            # reset slide
            set_servo_pulse(slide, slide_right)
            state = "transit"

except KeyboardInterrupt:
    print("exit")
    robot.close()
    sys.exit(0)
