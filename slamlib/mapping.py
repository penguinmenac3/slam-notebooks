import math
import numpy as np

from dataset import Dataset
from rangefinder import Laserscanner, Scan
from pose import Pose

class DensityGridMap(object):
    def __init__(self, size, global_offset_pose, scale=0.1, max_value=1, min_value=-1,
                 positive_scan_weight=0.1, negative_scan_weight=-0.02, min_dist=0.5, max_dist=100):
        self.size = size
        self.scale = scale
        self.max_value = max_value
        self.min_value = min_value
        self.offset_pose = global_offset_pose
        self.positive_scan_weight = positive_scan_weight
        self.negative_scan_weight = negative_scan_weight
        self.min_dist = min_dist
        self.max_dist = max_dist
        self.map = None
        self.reset_map()

    def reset_map(self):
        self.map = np.zeros((self.size, self.size))

    def add_density(self, pose, modificator):
        pose = self.offset_pose.clone().add_relative_pose(pose)
        tx = pose.x
        ty = pose.y
        ix = int(tx / self.scale)
        iy = int(ty / self.scale)

        if ix < 0 or ix >= self.size or iy < 0 or iy >= self.size:
            return False

        self.map[ix, iy] = max(min(modificator + self.map[ix, iy], self.max_value), self.min_value)

    def insert_scan_only_endpoint(self, scan, robot_pose):
        rays = scan.get_rays(robot_pose)

        for ray in rays:
            pose, dist = ray
            if dist < self.min_dist or dist > self.max_dist:
                continue

            end_pose = pose.clone().add_relative_pose(Pose(dist, 0, 0))
            self.add_density(end_pose, self.positive_scan_weight)

    def simulate_scan(self, scanner, robot_pose):
        # TODO implement
        pass
