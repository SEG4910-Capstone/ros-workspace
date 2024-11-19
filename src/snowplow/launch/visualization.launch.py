#/bin/python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    pkg_share = get_package_share_directory('snowplow')

    # Start GUI
    rviz_config_path = os.path.join(pkg_share, "config/rviz/navigation_config.rviz")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
    )
    print("Starting up rviz with config file: " + rviz_config_path)
    return LaunchDescription(
        [
            rviz_node,
        ]
    )


