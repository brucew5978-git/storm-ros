
import os

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource, PythonLaunchDescriptionSource

from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('drivetrain').find('drivetrain')

    slam_pkg = FindPackageShare('slam_toolbox').find('slam_toolbox')
    nav2_pkg = FindPackageShare('nav2_bringup').find('nav2_bringup')
    
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(slam_pkg, 'launch/online_async_launch.py')]),
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_pkg, 'launch/navigation_launch.py'),
            launch_arguments = {
                'params_file' : [os.path.join(pkg_share, 'config/nav2_config.yaml')]
            }.items(),
        )
    )

    return LaunchDescription([
        slam_launch,
        nav2_launch,
    ])
