import math
import numpy as np

class PosController:

    def __init__(self):
        self.dist_p = 0.5
        self.turn_p = 100
        self.spin_p = 0.07

        self.min_speed = 0.015 # m/s
        self.max_speed = 0.3 # m/s
        self.min_radius = 0.02 # meters ( wheel base diameter)
        self.max_radius = 2.0 # meters
        self.dist_precision = 0.005 # meters
        self.heading_precision = 0.1 # degrees
        self.transit_angle = 5 # degrees
        self.fine_tune_dist = 0.025 # meters

    def _spd_lim(self,spd):
        return np.sign(spd) * min(max(abs(spd),self.min_speed),self.max_speed)

    def _rad_lim(self,rad):
        return np.sign(rad) * min(max(abs(rad),self.min_radius),self.max_radius)

    def set(self,target_x, target_y, target_heading):
        self.target_x = target_x
        self.target_y = target_y
        self.target_heading = target_heading

    def update(self, current_x, current_y, current_heading):

        dist_error = math.sqrt((self.target_x-current_x)**2+(self.target_y-current_y)**2)
        transit_heading = math.atan2(self.target_y-current_y,self.target_x-current_x)
        transit_heading_error = (transit_heading + math.pi) % (math.pi*2) - math.pi - current_heading
        heading_error = self.target_heading - current_heading
        heading_error = (heading_error + math.pi) % (math.pi*2) - math.pi

        print("dist: {}m, heading: {}, tar_heading: {}, heading_error: {}".format(round(dist_error,2),round(math.degrees(current_heading),2),round(math.degrees(self.target_heading),2),round(math.degrees(heading_error),2)))

        # We are at our location
        if dist_error < self.dist_precision:
            # We are pointing in the right direction
            if abs(heading_error) <= math.radians(self.heading_precision):
                return 0,0, True
            # We are not pointing in the right direction -> spin in place
            else:
                spin_speed = self._spd_lim(heading_error * self.spin_p )
                return spin_speed, None, False

        # we have a large heading error
        elif abs(transit_heading_error) > math.radians(self.transit_angle):
            # Allow for reverse adjustments when we are close
            if dist_error < self.fine_tune_dist:
                linear_speed = self._spd_lim(-dist_error * self.dist_p)
                radius = self._rad_lim(self.turn_p/(math.pi - transit_heading_error))
                return linear_speed, radius, False
            # We are far away so we spin in place
            else:
                spin_speed = self._spd_lim(transit_heading_error * self.spin_p)
                return spin_speed, None, False

        # We are in transit to location
        else:
            linear_speed = self._spd_lim(dist_error * self.dist_p)
            if transit_heading_error != 0:
                radius = self._rad_lim(self.turn_p/transit_heading_error)
            else:
                radius = 0
            return linear_speed, radius, False

if __name__=="__main__":
    import time
    from open_interface import Roomba
    from pos_filter import PosFilter

    robot = Roomba("COM10")
    pos_filter = PosFilter(encoder_weight=1.0)
    control = PosController()

    filter_rate = 30
    control_rate = 30
    last_filter = time.time()
    last_control = time.time()

    arrived = False
    x,y,heading = 0,0,0
    set_x, set_y, set_heading = input("Enter location (x, y, heading): ").split(',')
    set_x = float(set_x)
    set_y = float(set_y)
    set_heading = math.radians(float(set_heading))
    control.set(set_x,set_y,set_heading)
    while not arrived:
        if time.time() - last_filter > 1.0/filter_rate:
            enc_left, enc_right = robot.encoders()
            x,y,heading = pos_filter.update(enc_left, enc_right,0,0)
            last_filter = time.time()

        if time.time() - last_control > 1.0/control_rate:
            speed, rad, arrived = control.update(x,y,heading)
            robot.drive(speed,rad)
            last_control = time.time()
