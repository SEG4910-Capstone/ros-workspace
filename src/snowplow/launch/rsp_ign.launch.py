import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node

import xacro


def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('snowplow'))
    xacro_file = os.path.join(pkg_path,'description','robot.urdf_ign.xacro')
    map_file = os.path.join(pkg_path, 'worlds', 'test.sdf')

    robot_description_config = Command(['xacro ', xacro_file, ' use_ros2_control:=', use_ros2_control, ' sim_mode:=', use_sim_time])
    robot_description = os.path.join(pkg_path,'description','robot_ign.sdf')

    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', '-v', '4', map_file],
        output='screen'
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        name='snowplow',
        arguments=[
            '-name', 'snowplow',
            '-file', robot_description,
            '-x', '0','-y', '0','-z', '1'
        ],
        output="screen"
    )
    # spawn = Node( package='ros_gz_sim', executable='create', 
    #             arguments=[ '-name', 'snowplow', '-topic', 'robot_description', ], 
    #             output='screen') 

    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        DeclareLaunchArgument(
            'use_ros2_control',
            default_value='true',
            description='Use ros2_control if true'),
        # ExecuteProcess(
        # cmd=['ro2','run','xacro','xacro', xacro_file, 'use_ros2_control:=' + use_ros2_control,
        #     'sim_mode:=' + use_sim_time, '-o', robot_description],
        # output='screen',
        # shell=True
        # ),   
        node_robot_state_publisher,
        gazebo,
        spawn_entity
    ])
