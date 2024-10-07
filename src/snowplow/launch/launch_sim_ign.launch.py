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
    params = {'robot_description': doc.toxml(), 'use_sim_time': True, 'use_ros2_control:=': True, 'sim_mode:=': True}

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
                    '-x', '0',
                   '-y', '0',
                   '-z', '1']
    )
    
    # Gazebo 
    # This doesn't work. Tried to look for how to pass the yaml file, but not much luck so just converting the cmd line script directly to a ros launch format
    # bridge = Node(
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge',
    #     arguments=[bridge_config],
    #     # arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
    #     output='screen'
    # )

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

    # New method of spawning the controllers
    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])

    controller_params_file = os.path.join(package_directory,'config','my_controllers.yaml')

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_description},
                    controller_params_file]
    )

    delayed_controller_manager = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=ignition_spawn_entity,
            on_exit=controller_manager
        )
    )
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_broad_spawner,
            on_exit=[diff_drive_spawner],
        )
    )



    # Launch them all!
    return LaunchDescription([
        bridge,
        gazebo,
        delayed_controller_manager,
        delayed_joint_broad_spawner,
        delayed_diff_drive_spawner,
        node_robot_state_publisher,
        ignition_spawn_entity,
        joystick,
        twist_mux
    ])
