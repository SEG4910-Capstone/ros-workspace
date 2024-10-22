#/bin/python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.event_handlers import OnProcessExit

import xacro


def generate_launch_description():

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_share = get_package_share_directory('nav2_outdoor_example')
    bridge_config = os.path.join(pkg_share, 'config', 'gazebo_bridge_config.yaml')


    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.urdf_ign.xacro')
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)



    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': ' -r ' + pkg_share + '/world/empty.sdf'
        }.items(),
    )

    gz_sim_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', doc.toxml(),
                   '-name', 'snowplow',
                   '-allow_renaming', 'true',
                   '-z', '0.4'],
    )

    bridge = ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                '--ros-args', '-p', 'config_file:='+bridge_config
            ],
            output='screen'
        )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': doc.toxml(), 'use_sim_time': True}],
    )

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'diff_drive_base_controller'],
        output='screen'
    )

    gazebo_frame_modifier = Node(
        package='nav2_outdoor_example',
        executable='frame_id_modifier',
        output='screen',
    )
    return LaunchDescription(
        [
            robot_state_publisher_node,
            gz_sim,
            gz_sim_spawn_entity,
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=gz_sim_spawn_entity,
                    on_exit=[load_joint_state_controller],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=load_joint_state_controller,
                    on_exit=[load_joint_trajectory_controller],
                )
            ),
            bridge,
            # gazebo_frame_modifier
        ]
    )


