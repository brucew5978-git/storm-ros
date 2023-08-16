#! /home/odroid/code/catkin_ws/src/drivetrain/venv/bin/python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist 
from std_msgs.msg import String

import board
import busio
import time
import adafruit_motorkit
from adafruit_motor import stepper
import board
import math


WHEEL_DIAMETER = 0.1; # in metres
WHEEL_CIRUMFERENCE = WHEEL_DIAMETER * math.pi
WIDTH_ROBOT = 0.2

STEPS_PER_REV  = 200
SEC_TO_NS = 10**9


class DrivetrainNode(Node):

    def __init__(self):
        super().__init__('drivetrain')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
        self.subscription = self.create_subscription(
            String,
            'stop',
            self.stop_callback,
            10)
        self.subscription  # prevent unused variable warning

        i2c0 = busio.I2C(board.SCL, board.SDA)
        self.kit = adafruit_motorkit.MotorKit(i2c=i2c0)
        print("connected to board")

        self.right_direction = stepper.FORWARD
        self.left_direction = stepper.BACKWARD

        self.right_speed = 0
        self.left_speed = 0
        self.right_timer = self.create_timer(timer_period_sec=self.right_speed,callback=self.step_right)
        self.left_timer = self.create_timer(timer_period_sec=self.left_speed,callback=self.step_left)
        self.left_timer.cancel()
        self.right_timer.cancel()

        self.stopped = False

    def stop_callback(self, msg):
        if msg.data == 'STOP':
            self.get_logger().info('STOP called')
            self.stopped = not self.stopped


    #TODO rename this callback
    def cmd_vel_callback(self, msg):
        #self.get_logger().info('I heard: "x:%.3f, z:%.3f"' % (msg.linear.x, msg.angular.z))
        if self.stopped:
            vel = 0
            ang_vel = 0
        else:
            vel = msg.linear.x
            ang_vel = msg.angular.z
        self.convert_to_tank(vel=vel, ang_vel=ang_vel)
    
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
            self.set_speed(right_vel, left_vel)
    
    # returns period in seconds
    def rpm_to_period(self, rpm):
        freq = (rpm * STEPS_PER_REV) / 60.0
        return (1 / freq) * SEC_TO_NS 

    def set_speed(self, right,left):

        # convert m/s to rpm
        right_speed = (60.0 / WHEEL_CIRUMFERENCE) * right
        left_speed = (60.0 / WHEEL_CIRUMFERENCE) * left

        if right_speed < 0:
            self.right_direction = stepper.BACKWARD
        else:
            self.right_direction = stepper.FORWARD
        if left_speed < 0:
            self.left_direction = stepper.FORWARD
        else:
             self.left_direction = stepper.BACKWARD


        if right_speed == 0:
            self.right_timer.cancel()
        else:
            self.right_timer.reset()
            self.right_timer.timer_period_ns = self.rpm_to_period(right_speed)
        if left_speed == 0:
            self.left_timer.cancel()
        else:
            self.left_timer.reset()
            self.left_timer.timer_period_ns = self.rpm_to_period(left_speed)

    def step_right(self):
        #print("step right")
        self.kit.stepper1.onestep(direction=self.right_direction)

    def step_left(self):
       # print("step left")
        self.kit.stepper2.onestep(direction=self.left_direction)


def main(args=None):
    print('drivetrain main')
    print("making motor kit")

    rclpy.init(args=args)

    dt = DrivetrainNode()

    print("starting spin")
    rclpy.spin(dt)

    dt.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
