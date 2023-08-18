
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

    drivetrain_node = Node(
        package='drivetrain',
        executable='drivetrain',
        name='drivetrain'
    )
    
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_share, 'launch/imu_launch.py')]),
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(lidar_pkg, 'launch/ydlidar_launch.py')]),
    ) # to edit lidar launch settings, go to ~/ydlidar directory

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_share, 'launch/navigation_launch.py')]),
    )

    localization_node = Node(
        package='robot_localization',
        excecutable='ekf_node',
        name='ekf_filter',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config/ekf.yaml'),
                    {'use_sim_time' : True}]
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
    
    return LaunchDescription([
        drivetrain_node,
        imu_launch,
        lidar_launch,
        localization_node,
        navigation_launch,
        rosbridge_launch,
    ])
