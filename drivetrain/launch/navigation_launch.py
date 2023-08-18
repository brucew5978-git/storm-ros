
import os

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('drivetrain').find('drivetrain')

    slam_pkg = FindPackageShare('slam_toolbox').find('slam_toolbox')
    nav2_pkg = FindPackageShare('nav2_bringup').find('nav2_bringup')
    
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(slam_pkg, 'launch/online_async_launch.py')]),
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_pkg, 'launch/navigation_launch.py')),
        launch_arguments = {
            'params_file' : [os.path.join(pkg_share, 'config/nav2.yaml')]
        }.items(),
    ),

    tf2_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_base_footpring',
        arguments=['0.0', '0.0', '-0.05', '0', '0', '0', '0', 'base_link', 'base_footprint'], 
    ),


    return LaunchDescription([
        slam_launch,
        nav2_launch,
        tf2_node
    ])
