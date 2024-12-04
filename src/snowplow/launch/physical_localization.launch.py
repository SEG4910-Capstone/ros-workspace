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
                                    "config","slam", "physical_localization_params_online_async.yaml")
    
    rl_params_file = os.path.join(pkg_share, 
                                    "config","robot_localization", "physical_ekf_gps.yaml")

    map_transform_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_transform',
        output='screen',
        arguments = "--x -1 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 --frame-id map --child-frame-id odom".split(' '),
    )

    #  I believe we don't need this node as the SBG ellipse already fuses the IMU and GPS data to generate the odom
    # navsat_transform_node = Node(
    #     package='robot_localization',
    #     executable='navsat_transform_node',
    #     name='navsat_transform_node',
    #     output='screen',
    #     parameters=[{
    #         "magnetic_declination_radians": 0.0,
    #         "yaw_offset": 0.0,
    #         "zero_altitude": True,
    #         "use_odometry_yaw": False,
    #         "wait_for_datum": False,
    #         "publish_filtered_gps": False,
    #         "broadcast_utm_transform": False,
    #         "use_sim_time": False,
    #     }],
    #     remappings=[
    #         (controller_odom, '/odometry/filtered'),
    #         ("imu_plugin/out", "/imu"), # Input Imu
    #     ],
    #     arguments=['--ros-args', '--log-level', 'warn']
    # )
    ekf_odom = Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node_odom",
                output="screen",
                parameters=[rl_params_file, {"use_sim_time": False}],
                remappings=[("odometry/filtered", "odometry/local")],
            )
    
    # Might not need the ekf filter for the map if the SBG fused information is already accurate with minimal drift
    ekf_map = Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node_map",
                output="screen",
                parameters=[rl_params_file, {"use_sim_time": False}],
                remappings=[("odometry/filtered", "odometry/global")],
            )

    slam_toolbox = Node(
        parameters=[
          slam_file,
          {'use_sim_time': False}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen'
    )
    # Add a rviz node to visualize the map

    return LaunchDescription(
        [
            slam_toolbox,
            # navsat_transform_node,
            ekf_odom,
            # ekf_map,
            # map_transform_node
        ]
    )


