import numpy as np
import json

class Laserscanner(object):
    def __init__(self, angle_per_scan, number_of_scans, relative_to_base_link):
        self.angle_per_scan = angle_per_scan
        self.number_of_scans = int(number_of_scans)
        self.relative_to_base_link = relative_to_base_link

    def to_array(self):
        return [self.angle_per_scan, self.number_of_scans, self.relative_to_base_link.x, self.relative_to_base_link.y, self.relative_to_base_link.theta]

    def ray(self, base_link_pose, scan_ray_id):
        pose = base_link_pose.clone().add_relative_pose(self.relative_to_base_link)
        pose = pose.add_relative_pose(Pose(0, 0, self.angle_per_scan * (scan_ray_id - self.number_of_scans / 2.0)))
        return pose

    def to_str(self, padding=""):
        return '{\n  ' + padding + '"angle_per_scan": '+ str(self.angle_per_scan) +',\n  ' + padding + '"number_of_scans": '+ str(self.number_of_scans) + ',\n  ' + padding + '"relative_to_base_link": '+ self.relative_to_base_link.to_str() +'\n' + padding + '}'

class Scan(object):
    def __init__(self, time, data, scanner):
        self.time = time
        self.data = data
        self.scanner = scanner

    def to_array(self):
        return [self.time, self.data]

    def to_str(self, padding=""):
        return '{\n  ' + padding + '"time": ' + str(self.time) + ',\n  ' + padding + '"data": ' + str(self.data[:4]) + ',\n  ' + padding + '"scanner": ' + self.scanner.to_str("  " + padding) + '' + padding + '\n}'
