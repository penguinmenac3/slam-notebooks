import math
import numpy as np

from dataset import Dataset
from rangefinder import Laserscanner, Scan
from pose import Pose

class DensityGridMap(object):
    def __init__(self, size, global_offset_pose):
        self.size = size
        self.offset_pose = offset_pose

    def insert_scan(self, scan, robot_pose):
        # TODO implement
        pass

    def simulate_scan(self, scanner, robot_pose):
        # TODO implement
        pass
