import math

class Odometry:

    def __init__(self, robot):
        self.robot = robot
        self.global_pos = 0,0,math.pi


    def update():
        # read encoders
        left_encoder, right_encoder = self.robot.encoders()

        # straight
        if abs(left_encoder - right_encoder) < 1e-6:
            enc_delta_x = left_encoder * math.cos(self.heading)
            enc_delta_y = right_encoder * math.sin(self.heading)
            enc_delta_theta = 0
        # Not straight
        else:
            R = self.wheel_base * (left_encoder + right_encoder) / (2 * (right_encoder - left_encoder))
            enc_delta_theta = (right_encoder - left_encoder) / self.wheel_base

            enc_delta_x = R * math.sin(enc_delta_theta + self.heading) - R * math.sin(self.heading)
            enc_delta_y = - R * math.cos(enc_delta_theta + self.heading) + R * math.cos(self.heading)

        return enc_delta_x, enc_delta_y, enc_delta_theta
