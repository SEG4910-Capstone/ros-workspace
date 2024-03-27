import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart

from launch_ros.actions import Node



def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='snowplow' #<--- CHANGE ME

    rl_params_file = os.path.join(get_package_share_directory(package_name), 
                                  "config/robot_localization", "simulation_ekf_gps.yaml") # Change me for using different GPS params
    world = os.path.join(get_package_share_directory(package_name), 
                        "worlds", "capstone_1.world") #<--- Change map as required
    
    # Robot state publisher
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )

    joystick = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','joystick.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
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
                # remappings=[
                #     ("imu_plugin/out", "imu/data"),
                #     ("gps/fix", "gps/filtered_fix"),
                #     ("gps/filtered", "gps/filtered"),
                #     ("odometry/gps", "odometry/gps"),
                #     ("odometry/filtered", "odometry/global"),
                # ],
            )
    
    # The nav2 tutorial with gps launches the nav2 stack wit the following command. Think maybe this overlaps with the navigation_launch.py
    # https://github.com/ros-planning/navigation2_tutorials/blob/master/nav2_gps_waypoint_follower_demo/launch/gps_waypoint_follower.launch.py
    # navigation2_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(bringup_dir, "launch", "navigation_launch.py")
    #     ),
    #     launch_arguments={
    #         "use_sim_time": "True",
    #         "params_file": configured_params,
    #         "autostart": "True",
    #     }.items(),
    # )

    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': True}],
            remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
        )
    
    # Original method of launching gazebo
    gazebo_params_file = os.path.join(get_package_share_directory(package_name),'config','gazebo_params.yaml')

    # Include the Gazebo launch file, provided by the gazebo_ros package
    # gazebo = IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource([os.path.join(
    #                 get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    #                 launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items()
    #          )
    # The above method of launching gazebo causes some issues that I'm not sure how to resolve. For now, this will work.
    gazebo = ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world], output='screen'
            )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'snowplow'],
                        output='screen')

    # New method of spawning the controllers
    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])

    controller_params_file = os.path.join(get_package_share_directory(package_name),'config','my_controllers.yaml')

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_description},
                    controller_params_file]
    )

    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner],
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

    # Launch them all!
    return LaunchDescription([
        rsp,
        joystick,
        twist_mux,
        gazebo,
        spawn_entity,
        delayed_controller_manager,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner,
        ekf_odom,
        ekf_map,
        navsat
    ])
