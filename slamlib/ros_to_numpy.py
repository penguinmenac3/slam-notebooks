import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

class RosToNumpy(object):
    def __init__(self, args):
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.on_odometry)
        self.front_lidar_sub = rospy.Subscriber("/lidar_front", LaserScan, self.on_laser_front)
        self.back_lidar_sub = rospy.Subscriber("/lidar_back", LaserScan, self.on_laser_back)

    def on_odometry(self, data):
        pass

    def on_laser_front(self, data):
        pass

    def on_laser_back(self, data):
        pass

    def kill(self):
        self.odom_sub.unsubscribe()
        self.front_lidar_sub.unsubscribe()
        self.back_lidar_sub.unsubscribe()

        # todo save


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
