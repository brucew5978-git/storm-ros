
import rclpy
from geometry_msgs.msg import Twist 

import time

PI = 3.1415926536
pace = 1 #factor used to adjust robot's true speed (pace = ros_vel/true_vel)

class Storm():
    
    def __init__(self):
        rclpy.init(args=None)
        self.node = rclpy.create_node('script_publisher')
        self.publisher = self.node.create_publisher(Twist, 'cmd_vel', 10)
    
    def set_vel(self, lin_vel, ang_vel, quiet=False):
        twist = Twist()
        twist.linear.x = float(lin_vel)
        twist.angular.z = float(ang_vel) 

        if not quiet: self.node.get_logger().info('Publishing: linear: %f m/s , angular: %f m/s' % (twist.linear.x, twist.angular.z))
        self.publisher.publish(twist)

    def circle(self, radius, speed=1):
        if radius <= 0:
            print('Error: radius must be a positive number')
            return
        if speed <= 0:
            print('Error: velocity must be a positive number')
            return

        print('Creating circle with radius %f' % radius)
        ang_vel = speed/radius
        self.set_vel(speed, ang_vel)
        d = 2*PI*radius
        time.sleep( d/speed )

    def drive(self, length, speed):
        if length <= 0:
            print('Error: distance must be a postive number')
            return

        print('Driving %f distance' % length)
        self.set_vel(speed, 0)
        time.sleep( abs(length/speed) )
        self.set_vel(0, 0)

    def turn(self, angle, ang_vel=1):
        self.turn_rad(angle*PI/180, ang_vel)

    def turn_rad(self, angle, ang_vel=1):
        if ang_vel == 0:
            print('Error: speed cannot be 0')
            return

        if angle <= 0:
            print('Error: angle should be a postive number')
            return

        print('Turning %f rad clockwise' % angle)
        self.set_vel(0, ang_vel)
        time.sleep( abs(angle/ang_vel) )
        self.set_vel(0, 0)

    def box(self, length, speed=1):
        if length <= 0:
            print('Error: box length must be a positive number')
            return

        print('Creating box of length %f' % length)
        for i in range(3):
            self.drive(length, speed)
            self.turn(90, speed)

    def close(self):
        print('Stopping Storm')
        self.set_vel(0, 0)
        self.node.destroy_node()
        rclpy.shutdown()

    
