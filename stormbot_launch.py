from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    LaunchDescription([
        Node(
            package='drivetrain',
            executable='drivetrain',
            name='drivetrain'
        ),
        Node(
            package='drivetrain',
            executable='safestop',
            name='safestop'
        )
    ])