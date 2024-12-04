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
    map_transform_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_transform',
        output='screen',
        arguments = "--x -1 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 --frame-id map --child-frame-id odom".split(' '),
        )
    
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

    return LaunchDescription(
        [
            ekf_odom,
            # ekf_map,
            navsat_transform_node,
            map_transform_node
        ]
    )


