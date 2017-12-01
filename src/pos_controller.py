import math
import numpy as np

class PosController:

    def __init__(self):
        self.dist_p = 0.5
        self.turn_p = 1
        self.spin_p = 0.05

        self.target_x = None
        self.target_y = None
        self.target_heading = None

        self.min_speed = 0.011 # m/s this causes angular inaccuracies. We need finer tune control
        self.max_speed = 0.25 # m/s
        self.min_radius = 0.02 # meters ( wheel base diameter)
        self.max_radius = 2.0 # meters
        self.dist_precision = 0.003 # meters 0.005
        self.heading_precision = 0.05 # degrees 0.1
        self.transit_angle = 7 # degrees
        self.fine_tune_dist = 0.05 # meters
        self.accel = 0.035 # velocity step

        self.last_spd = 0

    def _spd_lim(self,spd,turn=True):
        if not turn:
            max_spd = min(self.last_spd + self.accel, self.max_speed) # limit acceleration
        else:
            max_spd = self.max_speed
        lim = np.sign(spd) * min(max(abs(spd),self.min_speed),max_spd)
        self.last_spd = lim
        return lim

    def _rad_lim(self,rad):
        rad = np.sign(rad) * max(abs(rad),self.min_radius)
        if abs(rad) > self.max_radius:
            rad = 0
        return rad



    def set(self,target_x, target_y, target_heading):
        self.target_x = target_x
        self.target_y = target_y
        self.target_heading = target_heading

    def update(self, current_x, current_y, current_heading):

        dist_error = math.sqrt((self.target_x-current_x)**2+(self.target_y-current_y)**2)
        transit_heading = math.atan2(self.target_y-current_y,self.target_x-current_x)
        transit_heading_error = transit_heading - current_heading
        transit_heading_error = (transit_heading_error + math.pi) % (math.pi*2) - math.pi
        heading_error = self.target_heading - current_heading
        heading_error = (heading_error + math.pi) % (math.pi*2) - math.pi

        print("-----------------------------")
        print("dist: {}m, heading: {}".format(round(dist_error,3),round(math.degrees(current_heading),3)))
        print("point_heading: {}, heading_error: {}".format(round(math.degrees(self.target_heading),2),round(math.degrees(heading_error),2)))
        print("transit_heading: {}, heading_error: {}".format(round(math.degrees(transit_heading),3),round(math.degrees(transit_heading_error),3)))
        # We are at our location
        if dist_error < self.dist_precision:
            # We are pointing in the right direction
            if abs(heading_error) <= math.radians(self.heading_precision):
                #print("here1")
                return 0,0, True
            # We are not pointing in the right direction -> spin in place
            else:
                #print("here3")
                spin_speed = self._spd_lim(heading_error * self.spin_p)
                return spin_speed, None, False

        # we have a large heading error
        elif abs(transit_heading_error) > math.radians(self.transit_angle):
            # Allow for reverse adjustments when we are close
            if dist_error < self.fine_tune_dist:
                #print("here2")
                linear_speed = self._spd_lim(-dist_error * self.dist_p)
                radius = self._rad_lim(linear_speed / (self.turn_p*(math.pi - transit_heading_error)))
                return linear_speed, radius, False
            # We are far away so we spin in place
            else:
                #print("here4")
                spin_speed = self._spd_lim(transit_heading_error * self.spin_p)
                return spin_speed, None, False

        # We are in transit to location
        else:
            #print("here5")
            linear_speed = self._spd_lim(dist_error * self.dist_p,turn=False)
            if transit_heading_error != 0:
                radius = self._rad_lim(linear_speed / (self.turn_p*transit_heading_error))
            else:
                radius = 0
            return linear_speed, radius, False

if __name__=="__main__":
    import time
    from open_interface import Roomba
    from pos_filter import PosFilter
    from hedgehog import Hedgehog

    robot = Roomba("COM10")
    hh = Hedgehog("COM8")
    pos_filter = PosFilter(robot,hh,encoder_weight=1.0)
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
    pos_filter.align()
    while not arrived:
        if time.time() - last_filter > 1.0/filter_rate:
            x,y,heading = pos_filter.update()
            last_filter = time.time()

        if time.time() - last_control > 1.0/control_rate:
            speed, rad, arrived = control.update(x,y,heading)
            robot.drive(speed,rad)
            last_control = time.time()
