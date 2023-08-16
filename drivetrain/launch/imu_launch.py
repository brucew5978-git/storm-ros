
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    imureader = Node(
        package='drivetrain',
        executable='imureader',
        name='imu'
    )

    tf2_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_imu',
        arguments=['0', '0', '0.02', '0', '0', '0', '1', 'base_link', 'odom'],
    )
    
    return LaunchDescription([
        imureader,
        tf2_node,
    ])
