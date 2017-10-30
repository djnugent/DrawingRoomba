import math
import numpy as np

class PosFilter:

    def __init__(self,encoder_weight = 0.9):
        # Constants
        self.wheel_base = 0.235 # meters
        self.encoder_weight = encoder_weight # ratio

        # State
        self.x = 0 # meters
        self.y = 0 # meters
        self.heading = math.pi/2 #radians
        self.bec_last_x = 0 # last beacon x location in meters
        self.bec_last_y = 0 # last beacon y location in meters

    # Update filter
    def update(self,left_encoder, right_encoder, beacon_x, beacon_y):
        # Calculate encoder change in position(4 situations)
        enc_delta_x = 0
        enc_delta_y = 0
        enc_delta_theta = 0

        # straight
        if abs(left_encoder - right_encoder) < 1e-6:
            enc_delta_x = left_encoder * math.cos(self.heading)
            enc_delta_y = right_encoder * math.sin(self.heading)
            enc_delta_theta = 0
        else:
            R = self.wheel_base * (left_encoder + right_encoder) / (2 * (right_encoder - left_encoder))
            enc_delta_theta = (right_encoder - left_encoder) / self.wheel_base

            enc_delta_x = R * math.sin(enc_delta_theta + self.heading) - R * math.sin(self.heading)
            enc_delta_y = - R * math.cos(enc_delta_theta + self.heading) + R * math.cos(self.heading)


        # Calculate beacon heading
        bec_heading = math.atan2(beacon_x - self.bec_last_x,beacon_y - self.bec_last_y)
        self.bec_last_x = beacon_x
        self.bec_last_y = beacon_y

        # Convert encoder from relative position to relative using the last estimated position
        enc_x = self.x + enc_delta_x
        enc_y = self.y + enc_delta_y
        enc_heading = (self.heading + enc_delta_theta + math.pi) % (math.pi*2) - math.pi

        # Weighted sum of encoder and beacon
        self.x = (1-self.encoder_weight)*beacon_y + self.encoder_weight*enc_x
        self.y = (1-self.encoder_weight)*beacon_x + self.encoder_weight*enc_y
        self.heading = (1-self.encoder_weight)*bec_heading + self.encoder_weight*enc_heading

        return self.x,self.y,self.heading


if __name__=="__main__":

    import time
    from open_interface import Roomba
    robot = Roomba("COM10")
    pos_filter = PosFilter(encoder_weight=1.0)

    speed = 0.1 # m/s
    dist = 0.7 # meters
    angle = 60 # degrees
    update_rate = 40 # hz

    try:
        while True:

            print("Drive forward")
            enc_left, enc_right = robot.encoders()
            start_location = pos_filter.update(enc_left, enc_right,0,0)
            robot.drive(speed,0)
            while True:
                enc_left, enc_right = robot.encoders()
                x,y,heading = pos_filter.update(enc_left, enc_right,0,0)
                print("X: {}m, Y: {}m, heading: {}deg".format(round(x,2),round(y,2),round(math.degrees(heading),2)))

                if math.sqrt((start_location[0] - x)**2 + (start_location[1] - y)**2) > 1.0:
                    break
                time.sleep(1.0/update_rate)


            robot.drive(0,0)
            time.sleep(0.3)

            enc_left, enc_right = robot.encoders()
            pos_filter.update(enc_left, enc_right,0,0)

            print("Turn left")
            robot.drive(-speed,None)
            while True:
                enc_left, enc_right = robot.encoders()
                x,y,heading = pos_filter.update(enc_left, enc_right,0,0)
                print("X: {}m, Y: {}m, heading: {}deg".format(round(x,2),round(y,2),round(math.degrees(heading),2)))

                if abs(start_location[2]-heading) > math.pi/2.0:
                    break
                time.sleep(1.0/update_rate)

            robot.drive(0,0)
            time.sleep(0.3)
    finally:
        robot.close()
