#/bin/python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription,  DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    pkg_share = get_package_share_directory('snowplow')

    params_file = os.path.join(pkg_share, 'config/nav2/physical_nav2_params.yaml')

    param_substitutions = {
        'yaml_filename': os.path.join(pkg_share, 'worlds/slam_navmap.yaml')
        }

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True)

    # Start map server
    lifecycle_nodes = ['map_server']
    map_server_node = Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', 'info'])

    map_server_lifecycle_node = Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                arguments=['--ros-args', '--log-level', 'info'],
                parameters=[{'use_sim_time': False},
                            {'autostart': True},
                            {'node_names': lifecycle_nodes}])

    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')


    # Start navigation
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav2_bringup, 'launch/navigation_launch.py')),
        launch_arguments={'use_sim_time': 'False', 'params_file': params_file}.items(),
    )


    return LaunchDescription(
        [
            map_server_node,
            map_server_lifecycle_node,
            nav2_bringup_launch
        ]
    )


    # namespace = LaunchConfiguration('namespace')
    # use_sim_time = LaunchConfiguration('use_sim_time')
    # autostart = LaunchConfiguration('autostart')
    # params_file = LaunchConfiguration('params_file')
    # default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    # map_subscribe_transient_local = LaunchConfiguration('map_subscribe_transient_local')

    # # lifecycle_nodes = ['controller_server',
    # #                    'smoother_server',
    # #                    'planner_server',
    # #                    'behavior_server',
    # #                    'bt_navigator',
    # #                    'waypoint_follower',
    # #                    'velocity_smoother']
    # lifecycle_nodes = ['controller_server',
    #                    'planner_server',
    #                    'behavior_server',
    #                    'bt_navigator',
    #                    'waypoint_follower']
    # # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # # https://github.com/ros/geometry2/issues/32
    # # https://github.com/ros/robot_state_publisher/pull/30
    # # TODO(orduno) Substitute with `PushNodeRemapping`
    # #              https://github.com/ros2/launch_ros/issues/56
    # remappings = [('/tf', 'tf'),
    #               ('/tf_static', 'tf_static')]

    # # Create our own temporary YAML files that include substitutions
    # param_substitutions = {
    #     'use_sim_time': use_sim_time,
    #     'default_bt_xml_filename': default_bt_xml_filename,
    #     'autostart': autostart,
    #     'map_subscribe_transient_local': map_subscribe_transient_local}

    # configured_params = RewrittenYaml(
    #         source_file=params_file,
    #         root_key=namespace,
    #         param_rewrites=param_substitutions,
    #         convert_types=True)

    # return LaunchDescription([
    #     # Set env var to print messages to stdout immediately
    #     SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

    #     DeclareLaunchArgument(
    #         'namespace', default_value='',
    #         description='Top-level namespace'),

    #     DeclareLaunchArgument(
    #         'use_sim_time', default_value='false',
    #         description='Use simulation (Gazebo) clock if true'),

    #     DeclareLaunchArgument(
    #         'autostart', default_value='true',
    #         description='Automatically startup the nav2 stack'),

    #     DeclareLaunchArgument(
    #         'params_file',
    #         default_value=os.path.join(bringup_dir, 'config', 'nav2', 'nav2_params.yaml'),
    #         description='Full path to the ROS2 parameters file to use'),

    #     DeclareLaunchArgument(
    #         'default_bt_xml_filename',
    #         default_value=os.path.join(
    #             get_package_share_directory('nav2_bt_navigator'),
    #             'behavior_trees', 'navigate_w_replanning_and_recovery.xml'),
    #         description='Full path to the behavior tree xml file to use'),

    #     DeclareLaunchArgument(
    #         'map_subscribe_transient_local', default_value='false',
    #         description='Whether to set the map subscriber QoS to transient local'),

    #     Node(
    #         package='nav2_controller',
    #         executable='controller_server',
    #         output='screen',
    #         parameters=[configured_params],
    #         remappings=remappings),
    #     # Node(
    #     #         package='nav2_smoother',
    #     #         executable='smoother_server',
    #     #         name='smoother_server',
    #     #         output='screen',
    #     #         respawn_delay=2.0,
    #     #         parameters=[configured_params],
    #     #         remappings=remappings),
    #     Node(
    #         package='nav2_planner',
    #         executable='planner_server',
    #         name='planner_server',
    #         output='screen',
    #         parameters=[configured_params],
    #         remappings=remappings),

    #     # Recovery changed. nav2_recoveries is now nav2_behaviors but it doesnt
    #     # have a recoveries_server executable. Need to figure out what to do with this

    #     # Node(
    #     #     package='nav2_behaviors',
    #     #     executable='recoveries_server',
    #     #     name='recoveries_server',
    #     #     output='screen',
    #     #     parameters=[configured_params],
    #     #     remappings=remappings),

    #     Node(
    #         package='nav2_bt_navigator',
    #         executable='bt_navigator',
    #         name='bt_navigator',
    #         output='screen',
    #         parameters=[configured_params],
    #         remappings=remappings),
    #     Node(
    #         package='nav2_behaviors',
    #         executable='behavior_server',
    #         name='behavior_server',
    #         output='screen',
    #         respawn_delay=2.0,
    #         parameters=[configured_params],
    #         remappings=remappings),
    #     Node(
    #         package='nav2_waypoint_follower',
    #         executable='waypoint_follower',
    #         name='waypoint_follower',
    #         output='screen',
    #         parameters=[configured_params],
    #         remappings=remappings),
    #     # Node(
    #     #     package='nav2_velocity_smoother',
    #     #     executable='velocity_smoother',
    #     #     name='velocity_smoother',
    #     #     output='screen',
    #     #     respawn_delay=2.0,
    #     #     parameters=[configured_params],
    #     #     remappings=remappings +
    #     #             [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),
    #     Node(
    #         package='nav2_lifecycle_manager',
    #         executable='lifecycle_manager',
    #         name='lifecycle_manager_navigation',
    #         output='screen',
    #         parameters=[{'use_sim_time': use_sim_time},
    #                     {'autostart': autostart},
    #                     {'node_names': lifecycle_nodes}]),

    # ])

