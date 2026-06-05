cmake_minimum_required(VERSION 3.10)
project(base_transform_twist_publisher_gz_plugin)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-plugin1 REQUIRED)

add_library(BaseTransformTwistPublisher SHARED
  src/BaseTransformTwistPublisher.cpp
)

target_include_directories(BaseTransformTwistPublisher PUBLIC
  include
)

ament_target_dependencies(BaseTransformTwistPublisher
  rclcpp
  tf2_ros
  geometry_msgs
)
target_link_libraries(BaseTransformTwistPublisher
  ignition-gazebo6::ignition-gazebo6
  ignition-plugin1::ignition-plugin1
)

install(
  TARGETS BaseTransformTwistPublisher
  LIBRARY DESTINATION lib
)

ament_package()
