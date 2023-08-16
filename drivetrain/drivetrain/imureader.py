import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped

import sensors.imu as imu
import time

refresh_interval = 1/100

class IMUReaderNode(Node):

    def __init__(self):
        super().__init__('imureader')
        self.pose = PoseWithCovarianceStamped()
        self.twist = TwistWithCovarianceStamped()

        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            'odom/ori',
            10)
        self.twist_pub = self.create_publisher(
            TwistWithCovarianceStamped,
            'odom/ang_vel',
            10)
        
        self.pose.covariance = imu.poseCovariance
        self.twist.covariance = imu.twistCovariance

        self.pose.header.frame_id = "odom"
        self.twist.header.frame_id = "odom"

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

            self.pose.header.stamp = time.time()
            self.pose_pub.publish(self.pose)

            self.twist.header.stamp = time.time()
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
