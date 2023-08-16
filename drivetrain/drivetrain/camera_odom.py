
import rclpy
from rclpy.node import Node
import time

from geometry_msgs.msg import Twist
from camera_inference import get_pitch_yaw

refresh_interval = 1/100

class CameraOdometry(Node):

    def __init__(self):
        super().__init__('cameraOdometry')

        self.time = time.time()
        self.twist = Twist()

        self.twist_publisher = self.create_publisher(
            Twist,
            'rotation/camera_angle',
            10,
        )

        self.update_camera_angle()
    
    def update_camera_angle(self):
        t = time.time()

        pitch, yaw = get_pitch_yaw()

        self.twist.angular.x = pitch
        self.twist.angular.y = yaw
        self.twist.angular.z = 0

        self.twist_publisher.publish(self.twist)

        time.sleep(refresh_interval)

def main (args=None):

    print('creating camera odometry')
    rclpy.init(args=args)
    co = CameraOdometry()
    print('co spinning')
    rclpy.spin(co)

    co.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
