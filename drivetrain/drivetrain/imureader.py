import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
from sensors_msgs.msg import Imu

import sensors.imu as imu_driver
import time

update_frequency = 10 #in Hz

class IMUReaderNode(Node):

    def __init__(self):
        super().__init__('imureader')

        self.publisher = self.create_publisher(
            Imu,
            'odom/imu',
            10)

        self.timer = self.create_timer(1/update_frequency, self.update_angular_odom)
        self.t = time.time()

    def update_angular_odom(self):
        aX, aY, aZ, tempC, gX, gY, gZ = imu_driver.get_calibrated_data()
        dt = time.time() - self.t
        self.t = time.time()

        q = imu_driver.mahony_update(aX, aY, aZ, gX, gY, gZ, dt)

        imuData = Imu()
        imuData.header.frame_id = "odom"

        imuData.orientation_covariance = imu_driver.oriCovariance
        imuData.angular_velocity_covariance = imu_driver.angVelCovariance
        imuData.linear_acceleration_covariance = imu_driver.accelCovariance

        imuData.orientation.w = q[0]
        imuData.orientation.x = q[1]
        imuData.orientation.y = q[2]
        imuData.orientation.z = q[3]

        imuData.angular_velocity.x = gX
        imuData.angular_velocity.y = gY
        imuData.angular_velocity.z = gZ

        imuData.linear_acceleration.x = aX
        imuData.linear_acceleration.y = aY
        imuData.linear_acceleration.z = aZ

        self.publisher.publish(imuData)

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
