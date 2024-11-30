import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    pkg_share = get_package_share_directory('snowplow')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch/physical_simulation.launch.py'))    
    )

    visualization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch/visualization.launch.py'))
    )

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch/drivers/lidar_driver.launch.py')),
        launch_arguments={
                'params_file': os.path.join(pkg_share, 'config', 'ouster', 'driver_params.yaml'),
                'auto_start': 'True',
                'ouster_ns': 'ouster_ns'
        }.items(),
    )

    imu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch/drivers/sbg_ellipse.launch.py'))
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch/physical_slam_localization.launch.py'))
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch/physical_navigation.launch.py'))
    )
    return LaunchDescription(
        [
            imu,
            lidar,
            simulation,
            visualization,
            localization,
            navigation,
        ]
    )


