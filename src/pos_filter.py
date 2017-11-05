import math
import numpy as np

class PosFilter:

    def __init__(self,robot,hh,encoder_weight = 0.9):
        # sensors
        self.robot = robot
        self.hh = hh

        # Constants
        self.wheel_base = 0.235 # meters
        self.encoder_weight = encoder_weight # ratio

        # Filter State
        self.filter_x = 0 # meters
        self.filter_y = 0 # meters
        self.filter_heading = math.pi/2 #radians

        # Internal state for performance comparison
        self.odometry_x = 0
        self.odometry_y = 0
        self.odometry_heading = math.pi/2 #radians

        # Beacon state
        self.beacon_origin_x = None # X origin, need to calibrate to set
        self.beacon_origin_y = None # Y origin, need to calibrate to set
        self.beacon_rot_mat = None # Frame of reference rotation, need to calibrate to set
        self.beacon_last_x = None # last beacon x location in meters
        self.beacon_last_y = None # last beacon y location in meters
        self.beacon_heading = 0 # beacon heading in radians

    def calibrate(self,x0,y0,x1,y1):
        # Translate origin
        self.origin_offset_x = x0
        self.origin_offset_y = y0

        # Rotate
        #current_angle = math.atan2()


    def align(self):

        # Wait for valid location from beacon
        while self.hh.read()[3] is None:
            time.sleep(0.1)

        # Set starting location of encoders
        enc_origin_x, enc_origin_y = 0,0

        # Get starting location of beacon
        num_readings = 10.0
        avg_x, avg_y = 0,0
        cnt = 0
        while cnt < num_readings:
            x,y,z,unread = self.hh.read()
            if unread:
                avg_x += x/num_readings
                avg_y += y/num_readings
                cnt += 1
            time.sleep(0.1)
        self.beacon_origin_x = avg_x
        self.beacon_origin_y = avg_y

        # Move forward
        x,y,heading = 0,0,math.pi/2
        self.robot.drive(0.1,0)
        drive_time = 5
        start = time.time()
        while time.time()-start < drive_time:
            # Dead recon
            left,right = self.robot.encoders()
            diff_x, diff_y, diff_heading = self._odometry(left,right,heading)
            x+=diff_x
            y+=diff_y
            heading+=diff_heading

        # Register final encoder position
        self.robot.drive(0,0)
        time.sleep(0.2)
        left,right = robot.encoders()
        diff_x, diff_y, diff_heading = self._odometry(left,right,heading)
        x+=diff_x
        y+=diff_y
        heading+=diff_heading
        enc_final_x, enc_final_y = x,y

        # load filter
        self.filter_x = x
        self.filter_y = y
        self.odometry_x = x
        self.odometry_y = y


        # Register final beacon pos
        avg_x, avg_y = 0,0
        cnt = 0
        while cnt < num_readings:
            x,y,z,unread = self.hh.read()
            if unread:
                avg_x += x/num_readings
                avg_y += y/num_readings
                cnt += 1
            time.sleep(0.1)

        beacon_final_x, beacon_final_y = avg_x,avg_y
        self.beacon_last_x = avg_x
        self.beacon_last_y = avg_y

        # Calculate heading offset
        enc_angle = math.atan2(enc_final_y - enc_origin_y,enc_final_x - enc_origin_x)
        beacon_angle = math.atan2(beacon_final_y-self.beacon_origin_y,beacon_final_x-self.beacon_origin_x)
        angle_offset = enc_angle - beacon_angle

        print(self.beacon_origin_x,self.beacon_origin_y,self.beacon_last_x,self.beacon_last_y,math.degrees(angle_offset))


        # Calculate rotation matrix
        self.beacon_rot_mat = np.array((
                                    (math.cos(angle_offset),-math.sin(angle_offset)),
                                    (math.sin(angle_offset),math.cos(angle_offset))
                                    ))

    def _odometry(self,left_encoder, right_encoder, heading):
        enc_delta_x = 0
        enc_delta_y = 0
        enc_delta_theta = 0

        # straight
        if abs(left_encoder - right_encoder) < 1e-6:
            enc_delta_x = left_encoder * math.cos(heading)
            enc_delta_y = right_encoder * math.sin(heading)
            enc_delta_theta = 0
        # Turning
        else:
            R = self.wheel_base * (left_encoder + right_encoder) / (2 * (right_encoder - left_encoder))
            enc_delta_theta = (right_encoder - left_encoder) / self.wheel_base
            enc_delta_x = R * math.sin(enc_delta_theta + heading) - R * math.sin(heading)
            enc_delta_y = - R * math.cos(enc_delta_theta + heading) + R * math.cos(heading)

        return enc_delta_x, enc_delta_y, enc_delta_theta

    # Update filter
    def update(self):

        encoder_weight = self.encoder_weight

        # Read sensors
        left_encoder, right_encoder = self.robot.encoders()
        beacon_x,beacon_y,beacon_z,unread = self.hh.read()


        ## Calculate odometry from encoders using filtered heading
        enc_delta_x, enc_delta_y, enc_delta_theta = self._odometry(left_encoder,right_encoder,self.filter_heading)

        # Convert encoder from relative position to absolute using the last estimated position
        enc_x = self.filter_x + enc_delta_x
        enc_y = self.filter_y + enc_delta_y
        enc_heading = self.filter_heading + enc_delta_theta
        enc_heading = (enc_heading + math.pi) % (math.pi*2) - math.pi

        ## Integrate beacon reading if it is available
        if unread:
            # Align beacon into frame of reference
            print(beacon_x,beacon_y)
            beacon_x -= self.beacon_origin_x
            beacon_y -= self.beacon_origin_y
            print(beacon_x,beacon_y)
            beacon_x, beacon_y = np.array((beacon_x, beacon_y)).dot(self.beacon_rot_mat.T)

            # Calculate beacon heading
            self.beacon_heading = math.atan2(beacon_y - self.beacon_last_y,beacon_x - self.beacon_last_x)
            self.beacon_heading = (self.beacon_heading + math.pi) % (math.pi*2) - math.pi
            self.beacon_last_x = beacon_x
            self.beacon_last_y = beacon_y

        else:
            # Only update using the encoder values
            encoder_weight = 1.0


        # Complementary Filter
        self.filter_x = (1-encoder_weight)*beacon_x + encoder_weight*enc_x
        self.filter_y = (1-encoder_weight)*beacon_y + encoder_weight*enc_y
        self.filter_heading = (1-encoder_weight)*self.beacon_heading + encoder_weight*enc_heading

        ## DEBUG
        # Calculate position using only odometry for performance analysis
        enc_delta_x, enc_delta_y, enc_delta_theta = self._odometry(left_encoder,right_encoder,self.odometry_heading)
        self.odometry_x += enc_delta_x
        self.odometry_y += enc_delta_y
        self.odometry_heading += enc_delta_theta
        self.odometry_heading = (self.odometry_heading + math.pi) % (math.pi*2) - math.pi

        return self.filter_x,self.filter_y,self.filter_heading

    # Get odometry only estimate
    def get_odometry_est(self):
        return self.odometry_x, self.odometry_y,self.odometry_heading

    # Get beacon only estimate
    def get_beacon_est(self):
        return self.beacon_last_x, self.beacon_last_y, self.beacon_heading


if __name__=="__main__":

    import time
    from open_interface import Roomba
    from hedgehog import Hedgehog

    robot = Roomba("COM10")
    hh = Hedgehog("COM8")
    pos_filter = PosFilter(robot,hh,encoder_weight=0.9)

    speed = 0.1 # m/s
    update_rate = 40 # hz

    try:
        print("Running calibration")
        pos_filter.align()

        while True:
            print("Drive forward")
            start_location = pos_filter.update()
            robot.drive(speed,0)
            while True:
                x,y,heading = pos_filter.update()
                bec_x,bec_y,bec_heading = pos_filter.get_beacon_est()
                odo_x, odo_y,odo_heading = pos_filter.get_odometry_est()
                print("Filter - X: {}m, Y: {}m, heading: {}deg".format(round(x,2),round(y,2),round(math.degrees(heading),2)))
                print("Beacon - X: {}m, Y: {}m, heading: {}deg".format(round(bec_x,2),round(bec_y,2),round(math.degrees(bec_heading),2)))
                print("Odometry X: {}m, Y: {}m, heading: {}deg".format(round(odo_x,2),round(odo_y,2),round(math.degrees(odo_heading),2)))
                print("------------------------------------------")

                if math.sqrt((start_location[0] - x)**2 + (start_location[1] - y)**2) > 1.0:
                    break

                time.sleep(1.0/update_rate)


            robot.drive(0,0)
            time.sleep(0.3)
            pos_filter.update()

            print("Turn left")
            robot.drive(-speed,None)
            while True:
                x,y,heading = pos_filter.update()
                bec_x,bec_y,bec_heading = pos_filter.get_beacon_est()
                odo_x, odo_y,odo_heading = pos_filter.get_odometry_est()
                print("Filter - X: {}m, Y: {}m, heading: {}deg".format(round(x,2),round(y,2),round(math.degrees(heading),2)))
                print("Beacon - X: {}m, Y: {}m, heading: {}deg".format(round(bec_x,2),round(bec_y,2),round(math.degrees(bec_heading),2)))
                print("Odometry X: {}m, Y: {}m, heading: {}deg".format(round(odo_x,2),round(odo_y,2),round(math.degrees(odo_heading),2)))
                print("------------------------------------------")
                if abs(start_location[2]-heading) > math.pi/2.0:
                    break
                time.sleep(1.0/update_rate)

            robot.drive(0,0)
            time.sleep(0.3)
            '''
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
    '''
    finally:
        robot.close()
