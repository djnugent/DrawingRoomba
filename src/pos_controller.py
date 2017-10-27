import math
import numpy as np

def PosController:

    def __init__(self):
        self.dist_p = 1.0
        self.turn_p = 1.0
        self.spin_p = 1.0

        self.min_speed = 0.015 # m/s
        self.max_speed = 0.5 # m/s
        self.min_radius = 0.3 # meters ( wheel base diameter)
        self.max_radius = 2.0 # meters
        self.dist_precision = 0.005 # meters
        self.heading_precision = 0.01 # degrees
        self.transit_angle = 5 # degrees
        self.fine_tune_dist = 0.025 # meters

    def _spd_lim(self,spd):
        return np.sign(spd) * min(max(abs(spd),self.min_speed),self.max_speed)

    def _rad_lim(self,rad):
        return np.sign(rad) * min(max(abs(rad),self.min_radius),self.max_radius)

    def update(self, current_x, current_y, current_heading, target_x, target_y, target_heading):

        dist_error = math.sqrt((target_x-current_y)**2+(target_y-current_y)**2)
        transit_heading_error = math.atan(target_x-current_x,target_y-current_y)
        transit_heading_error = (transit_heading_error + 180) % 360 - 180
        heading_error = target_heading - current_heading
        heading_error = (heading_error + 180) % 360 - 180

        # We are at our location
        if dist_error < self.precision:
            # We are pointing in the right direction
            if abs(heading_error) <= math.radians(self.heading_precision):
                return 0,0, True
            # We are not pointing in the right direction -> spin in place
            else:
                spin_speed = self._spd_lim(heading_error * self.spin_p)
                return spin_speed, 1, False

        # we have a large heading error
        elif abs(transit_heading_error) > self.radians(self.transit_angle):
            # Allow for reverse adjustments when we are close
            if dist_error < self.fine_tune_dist:
                linear_speed = self._spd_lim(-dist_error * self.dist_p)
                radius = self._rad_lim((180 - transit_heading_error) * self.turn_p)
                return linear_speed, radius, False
            # We are far away so we spin in place
            else:
                spin_speed = self._spd_lim(transit_heading_error * self.spin_p)
                return spin_speed, 1, False

        # We are in transit to location
        else:
            linear_speed = self._spd_lim(dist_error * self.dist_p)
            radius = self._rad_lim(transit_heading_error * self.turn_p)
            return linear_speed, radius, False

if __name__=="__main__":
    import time
    from open_interface import Roomba
    from pos_filter import PosFilter

    robot = Roomba("COM10")
    posf = PosFilter()
    control = PosController()

    filter_rate = 50
    control_rate = 25
    last_filter = time.time()
    last_control = time.time()

    arrived = False
    north,east,heading = 0,0,0
    set_north, set_east, set_heading = input("Enter location (north, east, heading): ").split(',')

    while not arrived:
        if time.time() - last_filter > 1.0/filter_rate:
            enc_left, enc_right = robot.encoders()
            north,east,heading = pos_filter.update(enc_left, enc_right,0,0)
            last_filter = time.time()

        if time.time() - last_control > 1.0/control_rate:
            speed, rad, arrived = control.update(north,east,heading,set_north,set_east,set_heading)
            robot.drive(speed,rad)
            last_control = time.time()
