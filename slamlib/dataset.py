import sys
import numpy as np
from pose import Pose
from rangefinder import Laserscanner, Scan

class Playback(object):
    def __init__(self, dataset, on_scan, on_pose):
        self.on_scan = on_scan
        self.on_pose = on_pose
        self.dataset = dataset

        # Initialize internal state
        self.last_pose_id = -1
        self.last_scan_id = {}
        self.last_scanner = None
        for scanner in self.dataset.get_scanners():
            self.last_scan_id[scanner] = -1

    def _inc_pose(self):
        self.last_pose_id = self.last_pose_id + 1

    def _inc_scanner(self):
        self.last_scan_id[self.last_scanner] = self.last_scan_id[self.last_scanner] + 1

    def tick(self):
        lowest_time = sys.maxint
        lowest = None
        lowest_method = None
        lowest_inc = None

        candidate = self.dataset.get_pose(self.last_pose_id + 1)
        if candidate is not None:
            lowest_time = candidate[0]
            lowest = candidate
            lowest_method = self.on_pose
            lowest_inc = self._inc_pose

        for scanner in self.dataset.get_scanners():
            candidate = self.dataset.get_laserscan(scanner, self.last_scan_id[scanner] + 1)
            if candidate is not None and lowest_time > candidate.time:
                lowest_time = candidate.time
                lowest = candidate
                lowest_method = self.on_scan
                lowest_inc = self._inc_scanner
                self.last_scanner = scanner

        if lowest_method is not None and lowest is not None:
            lowest_method(lowest)
            lowest_inc()
            return True
        else:
            return False

class Dataset(object):
    def __init__(self):
        self.data = {}
        self.data["scanners"] = []
        self.data["poses"] = []
        self.scanners = {}

    def load(self, dataset):
        self.data = dict(np.load(dataset))
        for x in self.data["scanners"]:
            info = self.data["scanner_" + str(x)]
            self.scanners[x] = Laserscanner(info[0], info[1], Pose(info[2], info[3], info[4]))

    def save(self, dataset):
        self.data["scanners"] = []
        for x in self.scanners:
            self.data["scanners"].append(x)
            self.data["scanner_" + str(x)] = np.array(self.scanners[x].to_array())

        if len(self.data["scanners"]) == 1:
            np.savez(dataset, poses=self.data["poses"], scanners=self.data["scanners"], scanner_0=self.data["scanner_0"], scanner_0_scans=self.data["scanner_0_scans"])
        elif len(self.data["scanners"]) == 2:
            np.savez(dataset, poses=self.data["poses"], scanners=self.data["scanners"], scanner_0=self.data["scanner_0"], scanner_0_scans=self.data["scanner_0_scans"], scanner_1=self.data["scanner_1"], scanner_1_scans=self.data["scanner_1_scans"])

    def add_scanner(self, scanner):
        scanner_id = str(len(self.scanners))
        self.scanners[scanner_id] = scanner
        self.data["scanner_" + scanner_id + "_scans"] = []

    def add_laserscan(self, scanner_id, scan):
        self.data["scanner_" + str(scanner_id) + "_scans"].append(scan.to_array())

    def add_pose(self, time, pose):
        self.data["poses"].append(np.array([time, pose.to_array()]).flatten())

    def get_pose(self, pose_id):
        if (pose_id >= len(self.data["poses"])):
             return None

        tmp = self.data["poses"][pose_id]
        time = tmp[0]
        pose = Pose(tmp[1][0], tmp[1][1], tmp[1][2])
        return (time, pose)

    def get_scanners(self):
        return self.data["scanners"]

    def get_scanner_info(self, scanner_id):
        return self.scanners[str(scanner_id)]

    def get_laserscan_number(self, scanner_id):
        return len(self.data["scanner_" + str(scanner_id) + "_scans"])

    def get_laserscan(self, scanner_id, scan_id):
        if (scan_id >= self.get_laserscan_number(scanner_id)):
             return None
        return Scan(self.data["scanner_" + str(scanner_id) + "_scans"][scan_id][0], self.data["scanner_" + str(scanner_id) + "_scans"][scan_id][1], self.get_scanner_info(scanner_id))
