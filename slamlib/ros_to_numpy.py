import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import tf

from dataset import Dataset
from rangefinder import Laserscanner, Scan
from pose import Pose

DATASET = "../slam_dataset.npz"
RESOLUTION_FRONT_LIDAR = 1081
RESOLUTION_BACK_LIDAR = 811
OFFSET_FRONT_LIDAR = Pose(0, 0, 0)
OFFSET_BACK_LIDAR = Pose(0, 0, math.pi)

class RosToNumpy(object):
    def __init__(self):
        self.dataset = Dataset()
        self.lidar_front = Laserscanner(math.radians(270.0) / float(RESOLUTION_FRONT_LIDAR), RESOLUTION_FRONT_LIDAR, OFFSET_FRONT_LIDAR)
        self.lidar_back = Laserscanner(math.radians(270.0) / float(RESOLUTION_BACK_LIDAR), RESOLUTION_BACK_LIDAR, OFFSET_BACK_LIDAR)
        self.dataset.add_scanner(self.lidar_front)
        self.dataset.add_scanner(self.lidar_back)

        self.odom_sub = rospy.Subscriber("/odom/fused", Odometry, self.on_odometry)
        self.front_lidar_sub = rospy.Subscriber("/robot/lidar_front", LaserScan, self.on_laser_front)
        self.back_lidar_sub = rospy.Subscriber("/robot/lidar_back", LaserScan, self.on_laser_back)

    def on_odometry(self, data):
        quaternion = (data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
        pose = Pose(data.pose.pose.position.x, data.pose.pose.position.y, tf.transformations.euler_from_quaternion(quaternion)[2])
        time = data.header.stamp.to_sec()
        self.dataset.add_pose(time , pose)

    def on_laser_front(self, data):
        time = data.header.stamp.to_sec()
        scan = Scan(time, data.ranges, self.lidar_front)
        self.dataset.add_laserscan(0, scan)

    def on_laser_back(self, data):
        time = data.header.stamp.to_sec()
        scan = Scan(time, data.ranges, self.lidar_back)
        self.dataset.add_laserscan(1, scan)

    def kill(self):
        self.odom_sub.unregister()
        self.front_lidar_sub.unregister()
        self.back_lidar_sub.unregister()

        self.dataset.save(DATASET)


def main():
    rospy.init_node('ros_to_numpy', anonymous=False)
    rtn = RosToNumpy()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    rtn.kill()

if __name__ == '__main__':
    main()
