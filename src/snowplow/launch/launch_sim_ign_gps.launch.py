import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart, OnProcessExit

from launch_ros.actions import Node

import xacro

def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='snowplow' #<--- CHANGE ME
    package_directory = get_package_share_directory(package_name)
    
    map_file = os.path.join(package_directory, 'worlds', 'test.sdf')

    rl_params_file = os.path.join(get_package_share_directory(package_name), 
                                "config/robot_localization", "simulation_ekf_gps.yaml") # Change me for using different GPS params
    
    bridge_config = os.path.join(package_directory, 'config', 'gazebo_bridge_config.yaml')
    

    joystick = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    package_directory,'launch','joystick.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    twist_mux_params = os.path.join(package_directory,'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': True}],
            remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
    )
    

    xacro_file = os.path.join(package_directory,
                              'description',
                              'robot.urdf_ign.xacro')    
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml(), 'use_sim_time': True}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params],
    )

    ignition_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', doc.toxml(),
                   '-name', 'snowplow',
                   '-allow_renaming', 'true',
                   '-z', '0.6']
    )
    
    # Gazebo 
    bridge = ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                '--ros-args', '-p', 'config_file:='+bridge_config
            ],
            output='screen'
        )
    gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ros_ign_gazebo'),
                              'launch', 'ign_gazebo.launch.py')]),
            launch_arguments=[('gz_args', [' -r -v 4 '+ map_file])])

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_broad'],
        output='screen'
    )

    delayed_joint_state_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=ignition_spawn_entity,
            on_exit=[load_joint_state_controller]
        )
    )
    load_diff_drive_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'diff_cont'],
        output='screen'
    )
    delayed_diff_drive_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_controller,
            on_exit=[load_diff_drive_controller]
        )
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
    navsat = Node(
                package="robot_localization",
                executable="navsat_transform_node",
                name="navsat_transform",
                output="screen",
                parameters=[rl_params_file, {"use_sim_time": True}],
                remappings=[
                    ("imu", "imu/data"), # Input Imu
                    ("gps/fix", "gps/fix"), # Input NavSatFix
                    ("gps/filtered", "gps/filtered"),# Output NavSatFix
                    ("odometry/gps", "odometry/gps"), # Output Odometry
                    ("odometry/filtered", "odometry/global"), # Input Odometry
                ],
            )

    # Launch them all!
    return LaunchDescription([
        bridge,
        gazebo,
        delayed_joint_state_controller,
        delayed_diff_drive_controller,
        node_robot_state_publisher,
        ignition_spawn_entity,
        joystick,
        twist_mux,
        ekf_odom,
        ekf_map,
        navsat
    ])
