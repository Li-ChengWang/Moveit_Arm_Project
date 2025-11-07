cmake_minimum_required(VERSION 3.10)
project(mp_to_moveit_driver)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(mp_to_moveit_driver_node src/mp_to_moveit_driver_node.cpp)
ament_target_dependencies(mp_to_moveit_driver_node
  rclcpp geometry_msgs tf2_ros tf2_geometry_msgs moveit_ros_planning_interface)

install(TARGETS mp_to_moveit_driver_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
