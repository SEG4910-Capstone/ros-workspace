cmake_minimum_required(VERSION 3.5)
project(nav2_outdoor_example)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(frame_id_modifier src/frame_id_modifier.cpp)

ament_target_dependencies(frame_id_modifier rclcpp sensor_msgs)

install(TARGETS
  frame_id_modifier
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY  launch rviz config world urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()