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

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_share = get_package_share_directory('snowplow')
    bridge_config = os.path.join(pkg_share, 'config', 'gazebo_bridge_config.yaml')
    xacro_file = os.path.join(pkg_share, 'description', 'robot.urdf_ign.xacro')
    doc = xacro.process_file(xacro_file, mappings={'use_sim_time': 'true'})
    xacro.process_doc(doc)

    twist_mux_params = os.path.join(pkg_share,'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': True}],
            remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
    )


    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': ' -r ' + pkg_share + '/worlds/empty.sdf'
        }.items(),
    )

    gz_sim_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', doc.toxml(),
                   '-name', 'snowplow',
                   '-allow_renaming', 'true',
                   '-z', '0.15'],
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
        parameters=[{'robot_description': doc.toxml()}],
    )

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'diff_cont'],
        output='screen'
    )

    return LaunchDescription(
        [
            twist_mux,
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
        ]
    )


