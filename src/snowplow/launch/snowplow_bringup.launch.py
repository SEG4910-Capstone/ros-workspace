#/bin/python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    pkg_share = get_package_share_directory('nav2_outdoor_example')

    default_slam_toolbox_config = os.path.join(get_package_share_directory("nav2_outdoor_example"), 
                                    "config","slam", "localization_params_online_async.yaml")
    # slam_toolbox_arg = DeclareLaunchArgument(
    #     name="params_file",
    #     default_value=default_slam_toolbox_config,
    #     description="Parameter file location"
    # )
    
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch/simulation.launch.py'))
    )

    visualization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch/visualization.launch.py'))
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch/slam_localization.launch.py'))
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch/navigation.launch.py'))
    )

    return LaunchDescription(
        [
            simulation,
            visualization,
            localization,
            navigation,
        ]
    )


