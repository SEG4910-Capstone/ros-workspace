#/bin/python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression

from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions.declare_launch_argument import DeclareLaunchArgument

from launch.event_handlers import OnProcessExit

import xacro


def generate_launch_description():
    pkg_share = get_package_share_directory('snowplow')
    xacro_file = os.path.join(pkg_share, 'description', 'robot.urdf_ign.xacro')
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)

    joystick = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    pkg_share,'launch','joystick.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    twist_mux_params = os.path.join(pkg_share,'config','twist_mux.yaml')

    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': True}],
            remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': doc.toxml(), 'use_sim_time': False}],
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

    return LaunchDescription(
        [
            joystick,
            twist_mux,
            robot_state_publisher_node,
            load_joint_state_controller,
            load_joint_trajectory_controller
        ]
    )


