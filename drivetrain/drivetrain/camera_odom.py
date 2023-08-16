
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
        self.pitch = 0
        self.yaw = 0

        self.twist = Twist()

        self.twist_publisher = self.create_publisher(
            Twist,
            'rotation/camera_angle',
            10,
        )

        self.update_camera_angle()
    
    def update_camera_angle(self):
        time_difference = time.time() - self.time

        pitch, yaw = get_pitch_yaw()

        pitch_difference = pitch-self.pitch
        yaw_difference = yaw-self.yaw

        self.pitch = pitch
        self.yaw = yaw

        angular_velocity_x = (pitch_difference)/time_difference
        angular_velocity_y = (yaw_difference)/time_difference


        self.twist.angular.x = angular_velocity_x
        self.twist.angular.y = angular_velocity_y
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
