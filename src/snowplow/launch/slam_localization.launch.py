#/bin/python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    pkg_share = get_package_share_directory('snowplow')

    slam_file = os.path.join(pkg_share, 
                                    "config","slam", "localization_params_online_async.yaml")
    print(slam_file)
    rl_params_file = os.path.join(pkg_share, "config/robot_localization", "simulation_ekf_gps.yaml") 
    
    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[{
            "magnetic_declination_radians": 0.0,
            "yaw_offset": 0.0,
            "zero_altitude": True,
            "use_odometry_yaw": False,
            "wait_for_datum": False, # Might need to set this true for this to work with gps properly
            "publish_filtered_gps": False,
            "broadcast_utm_transform": False,
            "use_sim_time": True,
        }],
        remappings=[
            ('/odometry/filtered', '/odometry/local'), #Input odom # http://docs.ros.org/en/melodic/api/robot_localization/html/integrating_gps.html This doc is a bit outdated but the remapping is still the same
            # ("/imu", "/imu"), # Input Imu
            # ("/gps/fix", "/gps/fix")
        ],
        arguments=['--ros-args', '--log-level', 'warn']
    )
    ekf_odom = Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node_odom",
                output="screen",
                parameters=[rl_params_file, {"use_sim_time": True}],
                remappings=[("odometry/filtered", "odometry/local")],
            )
    ekf_map = Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node_map",
                output="screen",
                parameters=[rl_params_file, {"use_sim_time": True}],
                remappings=[("odometry/filtered", "odometry/global")],
            )

    slam_toolbox = Node(
        parameters=[
          slam_file,
          {'use_sim_time': True}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen'
    )

    return LaunchDescription(
        [
            slam_toolbox,
            ekf_odom,
            # ekf_map, # Havent really found a use for this yet 
            navsat_transform_node,
        ]
    )


