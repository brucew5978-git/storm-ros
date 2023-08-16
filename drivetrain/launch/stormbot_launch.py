
import os

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource, PythonLaunchDescriptionSource

from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('drivetrain').find('drivetrain')
    lidar_pkg = FindPackageShare('ydlidar_ros2_driver').find('ydlidar_ros2_driver')

    drivetrain = Node(
        package='drivetrain',
        executable='drivetrain',
        name='drivetrain'
    )
    
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_share, 'launch/imu_launch.py')]),
    )

    rosbridge_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('rosbridge_server'),
                'launch',
                'rosbridge_websocket_launch.xml'
                ]),
        ]),
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(lidar_pkg, 'launch/imu_launch.py')]),
    )
    
    return LaunchDescription([
        drivetrain,
        imu_launch,
        rosbridge_launch,
        lidar_launch,
    ])
