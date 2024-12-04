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
        'yaml_filename': os.path.join(pkg_share, 'worlds/slam/test.yaml'),
        'use_sim_time': 'True'
        }

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True)
    remappings = [('/tf', 'tf'),
                ('/tf_static', 'tf_static')]
    
    # lifecycle_nodes = ['controller_server',
    #                 'smoother_server',
    #                 'planner_server',
    #                 'behavior_server',
    #                 'bt_navigator',
    #                 'waypoint_follower',
    #                 'velocity_smoother']
    
    # load_composable_nodes = LoadComposableNodes(
    #     target_container='nav2',
    #     composable_node_descriptions=[
    #         ComposableNode(
    #             package='nav2_controller',
    #             plugin='nav2_controller::ControllerServer',
    #             name='controller_server',
    #             parameters=[configured_params],
    #             remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),
    #         ComposableNode(
    #             package='nav2_smoother',
    #             plugin='nav2_smoother::SmootherServer',
    #             name='smoother_server',
    #             parameters=[configured_params],
    #             remappings=remappings),
    #         ComposableNode(
    #             package='nav2_planner',
    #             plugin='nav2_planner::PlannerServer',
    #             name='planner_server',
    #             parameters=[configured_params],
    #             remappings=remappings),
    #         ComposableNode(
    #             package='nav2_behaviors',
    #             plugin='behavior_server::BehaviorServer',
    #             name='behavior_server',
    #             parameters=[configured_params],
    #             remappings=remappings),
    #         ComposableNode(
    #             package='nav2_bt_navigator',
    #             plugin='nav2_bt_navigator::BtNavigator',
    #             name='bt_navigator',
    #             parameters=[configured_params],
    #             remappings=remappings),
    #         ComposableNode(
    #             package='nav2_waypoint_follower',
    #             plugin='nav2_waypoint_follower::WaypointFollower',
    #             name='waypoint_follower',
    #             parameters=[configured_params],
    #             remappings=remappings),
    #         ComposableNode(
    #             package='nav2_velocity_smoother',
    #             plugin='nav2_velocity_smoother::VelocitySmoother',
    #             name='velocity_smoother',
    #             parameters=[configured_params],
    #             remappings=remappings +
    #                        [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),
    #         ComposableNode(
    #             package='nav2_lifecycle_manager',
    #             plugin='nav2_lifecycle_manager::LifecycleManager',
    #             name='lifecycle_manager_navigation',
    #             parameters=[{'use_sim_time': 'true',
    #                          'autostart': 'true',
    #                          'node_names': lifecycle_nodes}]),
    #     ],
    # )
    # return LaunchDescription(
    #     [
    #         load_composable_nodes
    #     ]
    # )
    lifecycle_nodes = ['controller_server',
                       'smoother_server',
                       'planner_server',
                       'behavior_server',
                       'bt_navigator',
                       'waypoint_follower']
    remappings = [('/tf', 'tf'),
                ('/tf_static', 'tf_static')]

    
    nav2_controller = Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[configured_params],
            remappings=remappings,
            arguments=['--ros-args', '--log-level', 'info'])

    nav2_planner = Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[configured_params],
            remappings=remappings)

    bt_navigator = Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[configured_params],
            remappings=remappings)

    waypoint_follower = Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[configured_params],
            remappings=remappings)

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
            nav2_controller,
            nav2_planner,
            bt_navigator,
            waypoint_follower,
            lifecycle_node,
            nav2_bringup_launch
        ]
    )
