#! /home/odriod/code/catkin_ws/src/drivetrain/venv/bin/python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import String


class SafeStop(Node):
    def __init__(self):
        super().__init__('safestop')
        self.stopped = False

        self.subscription = self.create_subscription(
            String,
            '/stop',
            self.callback,
            10)
        self.subscription

        # create publishers that span all proccesses
        self.stop_pubs = []
        self.stop_pubs.append( (self.create_publisher(Twist,'/cmd_vel', 10), Twist()) )

        # create quick-firing timer for future use
        self.timer = self.create_timer(0.01, self.stop_all)

    def stop_all(self):
        if self.stopped:
            for pub in self.stop_pubs:
                pub[0].publish(pub[1])

    def callback(self, msg):
        if msg.data == 'STOP':
            print('STOP called')
            self.stopped = not self.stopped


def main(args=None):
    print('Initializing SafeStop')

    rclpy.init(args=args)

    ss = SafeStop()

    print('SafeStop ready')
    rclpy.spin(ss)

    ss.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
