import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node 


def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="True",
        description="Flag for node to follow sim clock",
    )
    default_slam_toolbox_config = os.path.join(get_package_share_directory("snowplow"), 
                                    "config","slam", "mapper_params_online_async.yaml")
    slam_toolbox_arg = DeclareLaunchArgument(
        name="params_file",
        default_value=default_slam_toolbox_config,
        description="Flag for node to follow sim clock",
    )
    # slam_toolbox_config = os.path.join(get_package_share_directory("snowplow"), 
                                # "config","slam", "localization_params_online_async.yaml")


    slam_toolbox = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory("slam_toolbox"),'launch','online_async_launch.py'
                )]), launch_arguments={'use_sim_time':  LaunchConfiguration("use_sim_time"), 'params_file': LaunchConfiguration("params_file")}.items()
    )

    # Add a rviz node to visualize the map
    rviz = Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", os.path.join(get_package_share_directory("snowplow"), "config", "rviz", "slam.rviz")],
            )
    
    return LaunchDescription([
        slam_toolbox_arg,
        use_sim_time_arg,
        slam_toolbox,
        rviz
    ])