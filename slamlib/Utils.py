class Laserscanner(object):
    def __init__(self, angle_per_scan, number_of_scans, relative_to_base_link):
        self.angle_per_scan = angle_per_scan
        self.number_of_scans = number_of_scans
        self.relative_to_base_link = relative_to_base_link

    def to_array(self):
        return [self.angle_per_scan, self.number_of_scans, self.relative_to_base_link[0], self.relative_to_base_link[1], self.relative_to_base_link[2]]

    def ray(self, base_link_pose, scan_ray_id):
        # TODO
        return np.array([1, 0])

class Scan(object):
    def __init__(self, time, data, laserscanner):
        self.time = time
        self.data = data
        self.laserscanner = laserscanner

    def to_array(self):
        return np.array([self.time, self.data]).flatten()

class Pose(object):
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

    def to_array(self):
        return np.array([self.x, self.y, self.theta])

    def x(self):
        return self.x

    def y(self):
        return self.y

    def theta(self):
        return self.theta

class Dataset(object):
    def __init__(self):
        self.data = {}
        self.data["scanners"] = []
        self.data["poses"] = []
        self.scanners = {}

    def load(self, dataset):
        self.data = np.load(dataset)
        for x in self.data["scanners"]:
            info = self.data["scanner_" + str(x)]
            self.scanners[x] = Laserscanner(info[0], info[1], [info[2], info[3], info[4]])

    def save(self, dataset):
        self.data["scanners"] = []
        for x in self.scanners:
            self.data["scanners"].append(x)
            self.data["scanner_" + str(x)] = np.array(self.scanners[x].to_array())

        if len(self.data["scanners"]) == 1:
            np.savez(dataset, poses=self.data["poses"], scanners=self.data["scanners"], scanner_0=self.data["scanner_0"], scanner_0_scans=self.data["scanner_0_scans"])
        elif len(self.data["scanners"]) == 2:
            np.savez(dataset, poses=self.data["poses"], scanners=self.data["scanners"], scanner_0=self.data["scanner_0"], scanner_0_scans=self.data["scanner_0_scans"], scanner_1=self.data["scanner_1"], scanner_1_scans=self.data["scanner_1_scans"])

    def add_scanner(self, name, scanner):
        self.scanners[name] = scanner

    def add_laserscan(self, scanner_id, scan):
        self.data["scanner_" + str(scanner_id) + "_scans"].append(scan.to_array())

    def add_pose(self, time, pose):
        self.data["poses"].append(np.array([time, pose.to_array()]).flatten())

    def get_pose(self, pose_id):
        tmp = self.data["poses"][pose_id]
        time = tmp[0]
        pose = Pose(tmp[1], tmp[2], tmp[3])
        return (time, pose)

    def get_scanners(self):
        return self.data["scanners"]

    def get_scanner_info(self, scanner_id):
        return self.scanners[scanner_id]

    def get_laserscan_number(self, scanner_id):
        return len(self.data["scanner_" + str(scanner_id) + "_scans"])

    def get_laserscan(self, scanner_id, scan_id):
        return Laserscan(self.data["scanner_" + str(scanner_id) + "_scans"][scan_id][0], self.data["scanner_" + str(scanner_id) + "_scans"][scan_id][1:], self.get_scanner_info(scanner_id))
