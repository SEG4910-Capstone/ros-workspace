#/bin/python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    pkg_share = get_package_share_directory('snowplow')

    slam_toolbox_arg = DeclareLaunchArgument(
        name="slam_file",
        default_value=os.path.join(pkg_share, 
                                    "config","slam", "localization_params_online_async.yaml"),
        description="Parameter file location"
    )

    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="True",
        description="Flag for node to follow sim clock",
    )
    
    localization_arg = DeclareLaunchArgument(
        name="localization_file",
        default_value= os.path.join(pkg_share, 
                                    "config","robot_localization", "simulation_ekf_gps.yaml"),
        description="Parameter file location"
    )
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch/simulation.launch.py')),
        launch_arguments=[('use_sim_time', LaunchConfiguration("use_sim_time"))]
    )

    nav2_params_arg = DeclareLaunchArgument(
        name="params_file",
        default_value=os.path.join(pkg_share, 'config', 'nav2', 'nav2_params.yaml'),
        description="Parameter file location",
    )

    visualization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch/visualization.launch.py'))
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch/slam_localization.launch.py'),
        launch_arguments=[('slam_file', LaunchConfiguration("slam_file")),
                          ('localization_file', LaunchConfiguration("localization_file")), 
                          ('use_sim_time', LaunchConfiguration("use_sim_time"))])
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch/navigation.launch.py'),
        launch_arguments=[('params_file', LaunchConfiguration("params_file")), 
                          ('use_sim_time', LaunchConfiguration("use_sim_time"))])
    )

    return LaunchDescription(
        [
            slam_toolbox_arg,
            use_sim_time_arg,
            localization_arg,
            nav2_params_arg,
            simulation,
            visualization,
            localization,
            navigation,
        ]
    )


