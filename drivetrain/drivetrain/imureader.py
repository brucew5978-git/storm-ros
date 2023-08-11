import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, Twist

import sensors.imu as imu
import time

refresh_interval = 1/100

class IMUReaderNode(Node):

    def __init__(self):
        super().__init__('imureader')
        self.pose = Pose()
        self.twist = Twist()

        self.pose_pub = self.create_publisher(
            Pose,
            'odom_ori',
            10)
        self.twist_pub = self.create_publisher(
            Twist,
            'odom_ang_vel',
            10)

        self.update_angular_odom()

    def update_angular_odom(self):
        t = time.time()
        while True:
            aX, aY, aZ, tempC, gX, gY, gZ = imu.get_calibrated_data()
            dt = time.time() - t
            t = time.time()

            q = imu.mahony_update(aX, aY, aZ, gX, gY, gZ, dt)
            #roll, pitch, yaw = imu.get_RPY()

            self.pose.orientation.w = q[0]
            self.pose.orientation.x = q[1]
            self.pose.orientation.y = q[2]
            self.pose.orientation.z = q[3]

            self.twist.angular.x = gX
            self.twist.angular.y = gY
            self.twist.angular.z = gZ

            self.pose_pub.publish(self.pose)
            self.twist_pub.publish(self.twist)

            time.sleep(refresh_interval)

def main(args=None):

    print('creating imu reader')
    rclpy.init(args=args)
    ir = IMUReaderNode()
    print('ar spinning')
    rclpy.spin(ir)

    ir.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
