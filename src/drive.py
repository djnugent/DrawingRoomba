from oi import Roomba
import time
import numpy as np


def straight_mm(mm,speed=100):
    delay = 0.78
    start_x, start_y = robot.encoders()
    drive_time = abs(1.0 * mm/speed) - delay
    robot.drive(np.sign(mm) * abs(speed),0)
    time.sleep(drive_time)
    robot.drive(0,0)
    time.sleep(0.05)
    end_x, end_y = robot.encoders()
    print("requested {}mm, moved: {}mm".format(mm,0.5*(end_x-start_x) + 0.5*(end_y-start_y) ))

print("Connect")
robot = Roomba("COM10")
try:
    kp = 1
    pos = 0
    target = 50
    error = target-pos
    while(abs(error) > 0.05):
        left,right = robot.encoders()
        pos += (left + right)/2
        error = target-pos
        velocity = np.sign(error) * min(max(abs(error * kp),15),500)
        robot.drive(velocity,0)

    print(pos)

finally:
    robot.close()
