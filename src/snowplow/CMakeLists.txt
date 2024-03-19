cmake_minimum_required(VERSION 3.5)
project(snowplow)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

set(dependencies
    camera_info_manager  
    gazebo
    gazebo_dev
    gazebo_msgs
    gazebo_ros
    gazebo_ros_pkgs
    geometry_msgs
    image_transport
    cv_bridge
    nav_msgs
    rclcpp
    sensor_msgs
    std_msgs
    std_srvs
 )

# find dependencies
find_package(ament_cmake REQUIRED)
#  REALSENSE CAMERA GAZEBO PLUGIN SHARED LIBRARY
add_library(gazebo_ros_realsense_camera_plugin SHARED  src/gazebo_ros_realsense_camera.cpp
                                            src/realsense_ros_plugin.cpp)
ament_target_dependencies(gazebo_ros_realsense_camera_plugin ${dependencies})
target_link_libraries(gazebo_ros_realsense_camera_plugin ${GAZEBO_LIBRARIES})

#  VELODYNE LIDAR GAZEBO PLUGIN SHARED LIBRARY
add_library(gazebo_ros_velodyne_laser SHARED src/gazebo_ros_velodyne_laser.cpp)
ament_target_dependencies(gazebo_ros_velodyne_laser ${dependencies})

#  HECTOR GPS SENSOR GAZEBO PLUGIN SHARED LIBRARY
add_library(gazebo_ros_gps SHARED src/gazebo_ros_gps.cpp)
ament_target_dependencies(gazebo_ros_gps ${dependencies})

#  HECTOR imu SENSOR GAZEBO PLUGIN SHARED LIBRARY
add_library(gazebo_ros_imu SHARED src/gazebo_ros_imu.cpp)
ament_target_dependencies(gazebo_ros_imu ${dependencies})

# this world plugin is used to attach sensor suite model to given robot body name
add_library(sensor_base_attacher_plugin SHARED src/sensor_base_attacher_plugin.cpp)
ament_target_dependencies(sensor_base_attacher_plugin ${dependencies})

ament_export_libraries(gazebo_ros_realsense_camera_plugin 
                       gazebo_ros_velodyne_laser 
                       gazebo_ros_gps
                       gazebo_ros_imu
                       sensor_base_attacher_plugin)


# Link against the library
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS 
        gazebo_ros_realsense_camera_plugin
        gazebo_ros_velodyne_laser
        gazebo_ros_gps
        gazebo_ros_imu
        sensor_base_attacher_plugin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

install(
  DIRECTORY config description launch worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_package()
