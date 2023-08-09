import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose

import sensors.imu as imu
import time

class IMUReaderNode(Node):

    def __init__(self):
        super().__init__('imureader')
        self.publisher = self.create_publisher(
            Pose,
            '/orientation',
            10)

        self.read_RPY_stream()

    def read_RPY_stream(self):
        t = time.time()
        while True:
            aX, aY, aZ, tempC, gX, gY, gZ = imu.get_calibrated_data()
            dt = (time.time() - t)
            t = time.time()

            q = imu.mahony_update(aX, aY, aZ, gX, gY, gZ, dt)
            roll, pitch, yaw = imu.get_RPY()
            pose = Pose()

            pose.position.x = 0.0
            pose.position.y = 0.0
            pose.position.z = 0.0

            pose.orientation.w = q[0]
            pose.orientation.x = q[1]
            pose.orientation.y = q[2]
            pose.orientation.z = q[3]

            self.publisher.publish(pose)
            time.sleep(0.01)

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
