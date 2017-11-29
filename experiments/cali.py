from open_interface import Roomba
from pos_filter import PosFilter
from pos_controller import PosController
from hedgehog import Hedgehog
import time
import numpy as np
import signal
import sys
import math


# Connect to Roomba
robot = Roomba("COM5")

# Connect to Hedgehog
hh = Hedgehog("COM8")

# Create Position Filter
pos_filter = PosFilter(robot,hh,encoder_xy_weight=.95,encoder_heading_weight=1)


update_rate = 60 #hz
last_update = 0
cnt = 0

# Zero encoders
left,right = robot.encoders()
start_odo_x,start_odo_y = 0,0

# Zero HH
num_readings = 10.0
avg_x, avg_y = 0,0
cnt = 0
while cnt < num_readings:
    x,y,z,unread = hh.read()
    if unread:
        avg_x += x/num_readings
        avg_y += y/num_readings
        cnt += 1
    time.sleep(0.1)
start_bec_x = avg_x
start_bec_y = avg_y

end_odo_x = 0
end_odo_y = 0
heading = 0

# move
robot.drive(0.1,0)
start_time = time.time()
while time.time() - start_time < 70:
    if time.time() - last_update > 1.0/update_rate:
        left,right = robot.encoders()
        dx,dy,dh = pos_filter._odometry(left,right,heading)
        end_odo_x += dx
        end_odo_y += dy
        heading += dh
    else:
        time.sleep(0.3/update_rate)

robot.drive(0,0)
time.sleep(1)

# register final beacon pos
avg_x, avg_y = 0,0
cnt = 0
while cnt < num_readings:
    x,y,z,unread = hh.read()
    if unread:
        avg_x += x/num_readings
        avg_y += y/num_readings
        cnt += 1
    time.sleep(0.1)
end_bec_x = avg_x
end_bec_y = avg_y

odo_dist = math.sqrt((start_odo_x-end_odo_x)**2 + (start_odo_y - end_odo_y)**2)
bec_dist = math.sqrt((start_bec_x-end_bec_x)**2 + (start_bec_y - end_bec_y)**2)

print("Odo dist:",odo_dist,"m")
print("Bec dist:",bec_dist,"m")

'''
odo 7.957
bec 8.024
tape 7.966
