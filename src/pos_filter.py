import math
import numpy as np
import time
from homography import HomographySolver


class PosFilter:

    def __init__(self,robot,hh,bno,encoder_xy_weight = 0.9, encoder_heading_weight = 0.99):
        # sensors
        self.robot = robot
        self.hh = hh
        self.bno = bno

        # homography solver
        self.hs = HomographySolver(dim = 2)

        # Constants
        self.wheel_base = 0.235 # meters
        self.encoder_xy_weight = encoder_xy_weight # ratio
        self.encoder_heading_weight = encoder_heading_weight
        self.realign_dist = 2.0 # meters
        self.beacon_heading_velocity = .1
        self.beacon_alpha = 0.5

        # Filter State
        self.filter_x = 0 # meters
        self.filter_y = 0 # meters
        self.filter_heading = math.pi/2 #radians
        self.filter_history = []
        self.beacon_history = []
        self.unaligned_dist = 0

        # Internal state for performance comparison
        self.odometry_x = 0
        self.odometry_y = 0
        self.odometry_heading = math.pi/2 #radians

        # IMU state
        self.bno_offset = None

        # Beacon state
        self.bec_R = None # Beacon rotation matrix
        self.bec_t = None # Beacon translation matrix
        self.beacon_last_x = None # last beacon x location in meters
        self.beacon_last_y = None # last beacon y location in meters
        self.beacon_last_time = 0
        self.beacon_heading = 0 # beacon heading in radians


    def align(self):

        # Wait for valid location from beacon
        print("waiting for valid data...")
        while self.hh.read()[3] is None:
            time.sleep(0.1)
        print("Done")

        # Set starting location of encoders
        enc_origin_x, enc_origin_y = 0,0

        # Set BNO offset
        yaw, roll, pitch = self.bno.read_euler()
        self.bno_offset = self.filter_heading - math.radians(360 - yaw)


        num_readings = 60.0
        cnt = 0
        encoder_locs = np.zeros((num_readings,2))
        beacon_locs = np.zeros((num_readings,2))

        enc_x,enc_y,enc_heading = 0,0,math.pi/2
        bec_x,bec_y,bec_z = 0,0,0

        # Drive forward and log position from both sensors
        self.robot.drive(0.05,0)
        while cnt < num_readings:
            # read beacon
            bec_x,bec_y,bec_z,unread = self.hh.read()

            # Update encoder odometry
            left,right = self.robot.encoders()
            diff_x, diff_y, diff_heading = self._odometry(left,right,enc_heading)
            enc_x+=diff_x
            enc_y+=diff_y
            enc_heading+=diff_heading

            # Log locations
            if unread:
                encoder_locs[cnt] = np.array((enc_x,enc_y))
                beacon_locs[cnt] = np.array((bec_x,bec_y))
                cnt += 1

        self.robot.drive(0,0)

        # load filter
        self.filter_x = enc_x
        self.filter_y = enc_y
        self.odometry_x = enc_x
        self.odometry_y = enc_y
        self.beacon_last_x = bec_x
        self.beacon_last_y = bec_y

        # Solve for the transformation between coordinate systems
        self.bec_R,self.bec_t,rmse = self.hs.solve_transformation(beacon_locs,encoder_locs)
        print("Error:", rmse)


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

        encoder_xy_weight = self.encoder_xy_weight
        encoder_heading_weight = self.encoder_heading_weight

        # Read sensors
        left_encoder, right_encoder = self.robot.encoders()
        beacon_x,beacon_y,beacon_z,unread = self.hh.read()
        yaw, roll, pitch = self.bno.read_euler()

        ## Calculate odometry from encoders using filtered heading
        enc_delta_x, enc_delta_y, enc_delta_theta = self._odometry(left_encoder,right_encoder,self.filter_heading)

        # Convert encoder from relative position to absolute using the last estimated position
        enc_x = self.filter_x + enc_delta_x
        enc_y = self.filter_y + enc_delta_y
        enc_heading = self.filter_heading + enc_delta_theta
        enc_heading = (enc_heading + math.pi) % (math.pi*2) - math.pi

        # Offset IMU heading
        bno_heading = self.bno_offset + math.radians(360-yaw)
        bno_heading = (bno_heading + math.pi) % (math.pi*2) - math.pi

        ## Integrate beacon reading if it is available
        if unread:
            # Align beacon into frame of reference
            A = (self.bec_R*np.mat([[beacon_x, beacon_y]]).T + np.tile(self.bec_t, (1, 1))).T
            A = np.asarray(A)
            beacon_x, beacon_y = A[0]

            # scale
            scale = 0.99277168494516
            beacon_x *= scale
            beacon_y *= scale


            '''
            # Rolling average
            beacon_x = (1-self.beacon_alpha) * beacon_x + self.beacon_alpha*self.beacon_last_x
            beacon_y = (1-self.beacon_alpha) * beacon_y + self.beacon_alpha*self.beacon_last_y

            # Calculate heading
            d_t = time.time() - self.beacon_last_time
            d_dist = math.sqrt((self.beacon_last_x-beacon_x)**2 + (self.beacon_last_y-beacon_y)**2)
            velocity = d_dist/d_t
            if velocity > self.beacon_heading_velocity:
                self.beacon_heading = math.atan2(beacon_y - self.beacon_last_y,beacon_x - self.beacon_last_x)
                self.beacon_heading = (self.beacon_heading + math.pi) % (math.pi*2) - math.pi
            else:
                encoder_heading_weight = 1.0
            '''
            self.beacon_last_x = beacon_x
            self.beacon_last_y = beacon_y
            self.beacon_last_time = time.time()


        # Complementary Filter
        self.filter_x = (1-encoder_xy_weight)*beacon_x + encoder_xy_weight*enc_x
        self.filter_y = (1-encoder_xy_weight)*beacon_y + encoder_xy_weight*enc_y
        self.filter_heading = (1-encoder_heading_weight)*bno_heading + encoder_heading_weight*enc_heading

        # Log positions from beacons and encoders
        if unread:
            # update distance travelled since last alignment
            if len(self.filter_history) > 0:
                last_fil_x, last_fil_y = self.filter_history[-1]
                self.unaligned_dist += math.sqrt((self.filter_x - last_fil_x)**2 + (self.filter_y - last_fil_y)**2)

            # Log positions
            self.filter_history.append([self.filter_x,self.filter_y])
            self.beacon_history.append([self.beacon_last_x, self.beacon_last_y])

            # Check if we are due for realignment in order to cancel encoder drift
            if self.unaligned_dist >= self.realign_dist:
                # Calculate transformation between the coordinate systems
                R,t,rmse = self.hs.solve_transformation(np.array(self.filter_history), np.array(self.beacon_history))

                # Realign filter with absolute beacon position
                old_x, old_y = self.filter_x, self.filter_y
                A = (R*np.mat([[self.filter_x, self.filter_y]]).T + np.tile(t, (1, 1))).T
                A = np.asarray(A)
                self.filter_x, self.filter_Y = A[0]

                # DEBUG
                realignment_dist = math.sqrt((old_x - self.filter_x)**2 + (old_y - self.filter_y)**2)
                print(">>REALIGNMENT -- rmse:",rmse, "shift(m):",realignment_dist)


                # Clear history
                self.unaligned_dist = 0
                self.filter_history = []
                self.beacon_history = []


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
    pos_filter = PosFilter(robot,hh,encoder_weight=0.98)

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
