import rclpy

class storm():
    
    def __init__(self):
        rclpy.init(args=None)
        pub_node = rclpy.create_node('script_publisher')
        publisher = pub_node.create_plubisher(Twist, 'cmd_vel', 10)
    
