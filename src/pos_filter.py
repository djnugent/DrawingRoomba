from io import Roomba
import math
import numpy as np

class PosFilter:

    # NED
    # Heading is clockwise off North
    def __init__(self,encoder_weight = 0.9):
        # Constants
        self.wheel_base = 0.235 # meters
        self.encoder_weight = encoder_weight # ratio

        # State
        self.north = 0 # meters
        self.east = 0 # meters
        self.heading = 0 #radians
        self.bec_last_x = 0 # last beacon x location in meters
        self.bec_last_y = 0 # last beacon y location in meters

    # Update filter
    def update(self,left_encoder, right_encoder, beacon_x, beacon_y):
        # Calculate encoder change in position(4 situations)
        enc_delta_north = 0
        enc_delta_east = 0
        enc_delta_theta = 0

        # Straight
        if left_encoder == right_encoder:
            enc_delta_north = math.cos(self.heading) * left_encoder
            enc_delta_east = math.sin(self.heading) * left_encoder

        # Spin
        elif np.sign(left_encoder) == - np.sign(right_encoder):
            arc_length = 0.5*(abs(left_encoder) + abs(right_encoder))
            enc_delta_theta = np.sign() arc_length /( 0.5 * self.wheel_base)

        # right turn
        elif left_encoder > right_encoder:
            outer_radius = (self.wheel_base * left_encoder) / (left_encoder - right_encoder)
            inner_radius = outer_radius - self.wheel_base
            center_radius = (outer_radius + inner_radius) / 2.0

            enc_delta_theta = left_encoder / outer_radius

            enc_delta_north = center_radius * math.sin(enc_delta_theta)
            enc_delta_east = center_radius - center_radius*math.cos(enc_delta_theta)

            # transform x,y based on heading
            rot = np.array((
                            (math.sin(self.heading),math.cos(self.heading))
                            (math.cos(self.heading),-math.sin(self.heading))
                            ))
            enc_delta_east, enc_delta_north = np.array((enc_delta_east, enc_delta_north)).dot(rot.T)

        # left turn
        elif right_encoder > left_encoder:
            outer_radius = (self.wheel_base * right_encoder) / (right_encoder - left_encoder)
            inner_radius = outer_radius - self.wheel_base
            center_radius = (outer_radius + inner_radius) / 2.0

            enc_delta_theta = left_encoder / outer_radius

            enc_delta_north = center_radius * math.sin(enc_delta_theta)
            enc_delta_east = -center_radius + center_radius*math.cos(enc_delta_theta)

            enc_delta_theta *= -1

            # transform x,y based on heading
            rot = np.array((
                            (math.sin(self.heading),math.cos(self.heading))
                            (math.cos(self.heading),-math.sin(self.heading))
                            ))
            enc_delta_east, enc_delta_north = np.array((enc_delta_east, enc_delta_north)).dot(rot.T)


        # Calculate beacon heading
        bec_heading = math.atan2(beacon_x - self.bec_last_x,beacon_y - self.bec_last_y)
        self.bec_last_x = beacon_x
        self.bec_last_y = beacon_y

        # Convert encoder from relative position to relative using the last estimated position
        enc_north = self.north + enc_delta_north
        enc_east = self.east + enc_delta_east
        enc_heading = self.heading + enc_delta_theta

        # Weighted sum of encoder and beacon
        self.north = (1-self.encoder_weight)*beacon_y + self.encoder_weight*enc_north
        self.east = (1-self.encoder_weight)*beacon_x + self.encoder_weight*enc_east
        self.heading = (1-self.encoder_weight)*bec_heading + self.encoder_weight*enc_heading

        return north,east,heading


if __name__=="__main__":

    import time
    robot = Roomba("COM10")
    pos_filter = PosFilter(encoder_weight=1.0)

    speed = 0.2 # m/s
    dist = 1.0 # meters
    angle = 90 # degrees
    update_rate = 20 # hz

    try:
        while True:

            print("Drive forward")
            steps = (dist / speed) * update_rate
            robot.drive(speed,0)
            for i range(0,steps):
                enc_left, enc_right = robot.encoders()
                north,east,heading = pos_filter.update(enc_left, enc_right,0,0)
                print("North: {}m, East: {}m, heading: {}deg".format(round(north,2),round(east,2),round(math.degrees(heading),2))
                time.sleep(1.0/update_rate)
            robot.drive(0,0)
            time.sleep(0.3)

            enc_left, enc_right = robot.encoders()
            north,east,heading = pos_filter.update(enc_left, enc_right,0,0)

            print("Turn left")
            steps = math.radians(angle) * (0.235/2) / speed * update_rate
            robot.drive(-speed,1)
            for i range(0,steps):
                enc_left, enc_right = robot.encoders()
                north,east,heading = pos_filter.update(enc_left, enc_right,0,0)
                print("North: {}m, East: {}m, heading: {}deg".format(round(north,2),round(east,2),round(math.degrees(heading),2))
                time.sleep(1.0/update_rate)
            robot.drive(0,0)
            time.sleep(0.3)
    finally:
        robot.close()
