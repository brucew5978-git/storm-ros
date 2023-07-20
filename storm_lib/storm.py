import rclpy
from geometry_msgs.msg import Twist 

class Storm():
    
    def __init__(self):
        rclpy.init(args=None)
        self.node = rclpy.create_node('script_publisher')
        self.publisher = self.node.create_publisher(Twist, 'cmd_vel', 10)
    
    def set_vel(self, lin_vel, ang_vel):
        twist = Twist()
        twist.linear.x = lin_vel
        twist.angular.z = ang_vel 

        self.node.get_logger().info('Publishing: linear: %f m/s , angular: %f m/s' % (twist.linear.x, twist.angular.z))
        self.publisher.publish(twist)

    def close(self):
        self.node.destroy_node()
        rclpy.shutdown()

    
