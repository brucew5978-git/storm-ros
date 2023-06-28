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
WIDTH_ROBOT = 0.2

STEPS_PER_REV  = 200


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
    
    def convert_to_tank(self, vel, ang_vel):
        if vel == 0:
            right_vel = ang_vel * WIDTH_ROBOT / 2.0
            left_vel = -1 * right_vel
            self.set_speed(right_vel, left_vel)
        elif ang_vel == 0:
            self.set_speed(vel,vel)
        else:
            left_vel = vel - ang_vel * WIDTH_ROBOT / 2.0
            right_vel = vel + ang_vel * WIDTH_ROBOT / 2.0
    
    def rpm_to_period(self, rpm):
        freq = (rpm * STEPS_PER_REV) / 60.0
        return 1 / freq

    def set_speed(self, right,left):

        # convert m/s to rpm
        right_speed = (60.0 / WHEEL_CIRUMFERENCE) * right
        left_speed = (60.0 / WHEEL_CIRUMFERENCE) * left 
        if right_speed == 0:
            self.right_timer.cancel()
        else:
            self.right_timer.reset()
            self.right_timer.timer_period_ns(self.rpm_to_period(right_speed))
        if left_speed == 0:
            self.left_timer.cancel()
        else:
            self.left_timer.reset()
            self.left_timer.timer_period_ns(self.rpm_to_period(left_speed))

        self.left_timer.timer_period_ns(self.rpm_to_period(self.left_speed))


    def step_right(self):
        print("step right")
        self.kit.stepper1.onestep()

    def step_left(self):
        print("step left")
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
