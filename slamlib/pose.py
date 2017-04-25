import numpy as np
import math

class Pose(object):
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

    def to_array(self):
        return np.array([self.x, self.y, self.theta])

    def to_str(self):
        return '{"x":'+ str(self.x) +', "y":'+ str(self.y) + ',"theta":'+ str(self.theta) +'}'

    def add_relative_pose(self, pose):
        new_theta = pose.theta + self.theta
        if new_theta > math.pi:
            new_theta -= 2 * math.pi
        if new_theta < -math.pi:
            new_theta += 2 * math.pi

        delta_x = math.cos(self.theta) * pose.x - math.sin(self.theta) * pose.y
        delta_y = math.sin(self.theta) * pose.x + math.cos(self.theta) * pose.y

        self.x += delta_x
        self.y += delta_y
        self.theta = new_theta

        return self

    def find_relative_pose_to(self, other_pose):
        '''
            Calculate the relative pose from this to other_pose.
        '''
        x = other_pose.x - self.x
        y = other_pose.y - self.y

        dtheta = other_pose.theta - self.theta

        delta_x = math.cos(-self.theta) * x - math.sin(-self.theta) * y
        delta_y = math.sin(-self.theta) * x + math.cos(-self.theta) * y

        return Pose(delta_x, delta_y, dtheta)

    def length(self):
        '''
            Calculate the length of a pose.
        '''
        return math.sqrt(self.x * self.x + self.y * self.y)

    def clone(self):
        return Pose(self.x, self.y, self.theta)
