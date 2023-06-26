#! /home/odroid/code/catkin_ws/src/drivetrain/venv/bin/python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist 

import board
import busio
import time
import adafruit_motorkit
import board
import math


WHEEL_DIAMETER = 0.1; # in metres

WHEEL_CIRUMFERENCE = WHEEL_DIAMETER * math.pi


class DrivetrainNode(Node):

    def __init__(self):
        super().__init__('drivetrain')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        i2c0 = busio.I2C(board.SCL0, board.SDA0)
        self.kit = adafruit_motorkit.MotorKit(i2c=i2c0)

        self.right_speed = 0
        self.left_speed = 0
        self.right_timer = self.create_timer(timer_period_sec=self.right_speed,callback=self.step_right)
        self.left_timer = self.create_timer(timer_period_sec=self.left_speed,callback=self.step_left)

    #TODO rename this callback
    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%f"' % msg.linear.x)
        #TODO calculate the required left and right speeds
        # and call the set speed function 
    
    def calculate_rpm(self, vel, angle):
        rpm = vel
    def set_speed(self, right,left):
        self.right_speed = right
        self.left_speed = left 

        #TODO do some rpm conversion math to convert rpm to a timer period
        self.right_timer.timer_period_ns(right)
        self.left_timer.timer_period_ns(left)


    def step_right(self):
        if self.right_speed != 0:
            print("step right")
            self.kit.stepper1.onestep()
    def step_left(self):
        if self.left_speed != 0:
            print("step left")
            self.kit.stepper2.onestep()


    #TODO configure these to only run at their set rates
    def step():
        self.kit.stepper1.onestep()
        self.kit.stepper2.onestep()






def main(args=None):
    print('drivetrain main')
    print("making motor kit")
    rclpy.init(args=args)

    dt = DrivetrainNode()

    rate = dt.create_rate(frequency=850)

    rclpy.spin(dt)

#    x = 0
#    while(rclpy.ok()):
#        dt.step()
#        dt.get_logger().info('%d' % x)
#
#        rate.sleep()
         

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    dt.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
