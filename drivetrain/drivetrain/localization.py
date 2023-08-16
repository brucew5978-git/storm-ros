
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist

import time

# DEPRECATED - Localization is done with robot-localization package

# https://en.wikipedia.org/wiki/Covariance_matrix
# https://www.cuemath.com/algebra/covariance-matrix/
# represents the joint variability (ie randomness) of two variables, or the
# approximate strength of the linear relationship between the measurements
# C(i,j) = 0, i & j have no relationship
# diagonal C(i,i) represents measurement i's own uncertainty

# velocity in x, y, z, angular velocity about X axis, Y axis, Z axis
#                   vx  vy vz wx wy wz
twistCovariance = [0.05, 0, 0, 0, 0, 0, # vx
                   0, 0.05, 0, 0, 0, 0, # vy
                   0, 0, 0.05, 0, 0, 0, # vz
                   0, 0, 0, 0.05, 0, 0, # wx
                   0, 0, 0, 0, 0.05, 0, # wy
                   0, 0, 0, 0, 0, 0.05] # wz

# x, y, z, rotation about X axis, Y axis, Z axis
#                  x   y  z  rl  pt yw
poseCovariance = [0.05, 0, 0, 0, 0, 0, # x
                  0, 0.05, 0, 0, 0, 0, # y
                  0, 0, 0.05, 0, 0, 0, # z
                  0, 0, 0, 0.05, 0, 0, # roll
                  0, 0, 0, 0, 0.05, 0, # pitch
                  0, 0, 0, 0, 0, 0.05] # yaw

class LocalizationNode(Node):

    def __init__(self):
        super().__init__('localization')

        self.pose = Pose()
        self.twist = Twist()

        self.t = time.time()

        self.subscription = self.create_subscriber(
            Pose,
            'odom/ori',
            self.update_orientation,
            10)
        self.subscription = self.create_subscriber(
            Twist,
            'odom/ang_vel',
            self.update_twist_angular,
            10)
        #odom topics published by imureader.py

        self.subscription = self.create_subscriber(
            Twist,
            'cmd_vel',
            self.update_linear,
            10)
        
        self.subscription  # prevent unused variable warning
        self.publisher = self.create_publisher(
            Odometry,
            'odom',
            10)

    def create_odom_msg(self):
        odom = Odometry()
        # The pose should be specified in the coordinate frame given by header.frame_id
        # The twist should be specified in the coordinate frame given by the child_frame_id

        odom.header.stamp = time.time()
        odom.header.frame_id = "odom"
        odom.child_frame_id = 'base_link'

        odom.pose.pose = self.pose
        odom.pose.covariance = poseCovariance

        odom.twist.twist = self.twist
        odom.twist.covariance = twistCovariance

        self.publisher.publish(odom)
        return odom
    
    def update_orientation(self, msg):
        self.pose.orientation = msg.orientation # update aboslute pose to pose parent?

    def update_twist_angular(self, msg):
        self.twist.angular = msg.angular # update coordinate frame?

    def update_twist_linear(self, msg):
        dt = time.time() - self.t
        self.t = time.time()

        self.twist.linear = msg.linear # update coordinate frame?
        
        self.pose.position.x += msg.linear.x * dt
        self.pose.position.y += msg.linear.y * dt
        self.pose.position.z += msg.linear.z * dt


def main(args=None):

    print('creating localization node')
    rclpy.init(args=args)
    ln = LocalizationNode()
    print('ln spinning')
    rclpy.spin(ln)

    ln.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
