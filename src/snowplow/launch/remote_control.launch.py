import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_share = get_package_share_directory('snowplow')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch/physical_simulation.launch.py'))    
    )

    visualization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch/visualization.launch.py'))
    )

    map_transform_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_transform',
        output='screen',
        arguments = "--x -1 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 --frame-id map --child-frame-id odom".split(' '),
        )
    
    return LaunchDescription(
        [
            simulation,
            visualization,
            map_transform_node
        ]
    )