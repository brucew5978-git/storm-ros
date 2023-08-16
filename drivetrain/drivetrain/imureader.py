import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped

import sensors.imu as imu
import time

update_frequency = 10 #in Hz

class IMUReaderNode(Node):

    def __init__(self):
        super().__init__('imureader')

        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            'odom/ori',
            10)
        self.twist_pub = self.create_publisher(
            TwistWithCovarianceStamped,
            'odom/ang_vel',
            10)

        self.timer = self.create_timer(1/update_frequency, self.update_angular_odom)
        self.t = time.time()

    def update_angular_odom(self):
        aX, aY, aZ, tempC, gX, gY, gZ = imu.get_calibrated_data()
        dt = time.time() - self.t
        self.t = time.time()

        q = imu.mahony_update(aX, aY, aZ, gX, gY, gZ, dt)
        #roll, pitch, yaw = imu.get_RPY()

        poseS = PoseWithCovarianceStamped()
        twistS = TwistWithCovarianceStamped()

        poseS.header.frame_id = "odom"
        twistS.header.frame_id = "odom"

        poseS.pose.covariance = imu.poseCovariance
        twistS.twist.covariance = imu.twistCovariance

        poseS.pose.pose.orientation.w = q[0]
        poseS.pose.pose.orientation.x = q[1]
        poseS.pose.pose.orientation.y = q[2]
        poseS.pose.pose.orientation.z = q[3]

        twistS.twist.twist.angular.x = gX
        twistS.twist.twist.angular.y = gY
        twistS.twist.twist.angular.z = gZ

        #poseS.header.stamp = self.get_clock().now().to_msg()
        self.pose_pub.publish(poseS)

        #twistS.header.stamp = self.get_clock().now().to_msg()
        self.twist_pub.publish(twistS)


def main(args=None):

    print('creating imu reader')
    rclpy.init(args=args)
    ir = IMUReaderNode()
    print('ir spinning')
    rclpy.spin(ir)

    ir.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
