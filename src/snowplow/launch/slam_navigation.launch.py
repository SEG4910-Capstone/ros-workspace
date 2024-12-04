#/bin/python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

from launch_ros.actions import LoadComposableNodes 
from launch_ros.descriptions import ComposableNode

from launch.actions import IncludeLaunchDescription,  DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    pkg_share = get_package_share_directory('snowplow')

    params_file = os.path.join(pkg_share, 'config/nav2/nav2_params.yaml')
    
    param_substitutions = {
        'yaml_filename': os.path.join(pkg_share, 'worlds/competition/test.yaml'),
        'use_sim_time': 'True'
        }

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True)
    
    lifecycle_nodes = [
                       'filter_mask_server', 
                       'costmap_filter_info_server']
    remappings = [('/tf', 'tf'),
                ('/tf_static', 'tf_static')]
    
    filter_mask_server = Node(
            package='nav2_map_server',
            executable='map_server',
            name='filter_mask_server',
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[configured_params])
    
    costmap_filter_info_server = Node(
            package='nav2_map_server',
            executable='costmap_filter_info_server',
            name='costmap_filter_info_server',
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[configured_params])

    lifecycle_node = Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                arguments=['--ros-args', '--log-level', 'info'],
                parameters=[{'use_sim_time': True},
                            {'autostart': True},
                            {'node_names': lifecycle_nodes}])


    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    # Start navigation
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav2_bringup, 'launch/navigation_launch.py')),
        launch_arguments={'use_sim_time': 'True', 'params_file': params_file}.items(),
    )


    return LaunchDescription(
        [
            # nav2_controller,
            # nav2_planner,
            # bt_navigator,
            # waypoint_follower,
            filter_mask_server,
            costmap_filter_info_server,
            lifecycle_node,
            nav2_bringup_launch
        ]
    )
