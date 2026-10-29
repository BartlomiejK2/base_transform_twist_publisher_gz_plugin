cmake_minimum_required(VERSION 3.10)
project(base_odometry_publisher_gz_plugin)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-plugin1 REQUIRED)

add_library(BaseOdometryPublisher SHARED
  src/BaseOdometryPublisher.cpp
)

target_include_directories(BaseOdometryPublisher PUBLIC
  include
)

ament_target_dependencies(BaseOdometryPublisher
  rclcpp
  tf2_ros
  geometry_msgs
  nav_msgs
)
target_link_libraries(BaseOdometryPublisher
  ignition-gazebo6::ignition-gazebo6
  ignition-plugin1::ignition-plugin1
)

install(
  TARGETS BaseOdometryPublisher
  LIBRARY DESTINATION lib
)

ament_package()
