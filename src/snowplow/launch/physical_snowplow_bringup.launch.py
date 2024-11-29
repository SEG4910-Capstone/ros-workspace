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
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch/drivers/lidar_driver.launch')),
        launch_arguments={
                'sensor_hostname': 'os-122116000061.local',
                'proc_mask': 'IMG|PCL|IMU|SCAN', # use any combination of the 4 flags to enable or disable specific processors (Might be good to disable imu if not used)
                'lidar_frame': 'laser_frame',
                'sensor_frame': 'laser_frame',
                'imu_frame': 'laser_frame',
                'rviz': 'true',
                'point_cloud_frame': 'laser_frame',
                'max_range': '50',
        }.items(),
    )

    imu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch/drivers/imu_driver.launch'))
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


