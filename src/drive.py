from oi import Roomba
from pos_filter import PosFilter
from PosController import PosController
import time
import numpy as np
import signal
import sys


# Connect to Roomba
robot = Roomba("COM10")

# Connect to Hedgehog
hh = Hedgehog("COM8")

# Create Position Filter
pos_filter = PosFilter()

# Create Position Controller
pos_ctrl = PosController()

# Handle program termination gracefully
def signal_term_handler(signal, frame):
    robot.close()
    sys.exit(0)
signal.signal(signal.SIGTERM, signal_term_handler)

length = 10 # meters
width = 3 # meters
waypoints = np.array([[0, length, 90],
                     [width, length, 0],
                     [width,  0,  -90],
                     [0,  0,  180]])

cnt = 0
update_rate = 30 #hz
last_update = 0

while True:
    if time.time() - last_update > 1.0/update_rate:
        x,y,heading = pos_filter.update()
        speed, rad, arrived = control.update(x,y,heading)
        robot.drive(speed,rad)
        if arrived:
            pnt = waypoints[cnt%(len(waypoints))]
            pos_ctrl.set(pnt[0],pnt[1],pnt[2])
            cnt += 1
        last_update = time.time()
    else:
        time.sleep(0.3/update_rate)
