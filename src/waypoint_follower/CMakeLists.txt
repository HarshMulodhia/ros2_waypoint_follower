cmake_minimum_required(VERSION 3.8)
project(waypoint_follower)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

# Find robotics_control library
find_package(robotics_control REQUIRED)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

# Generate header with installed default waypoints path so the node can
# use an absolute installed path as the default parameter value.
set(WAYPOINTS_DEFAULT_PATH "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/config/waypoints_sample.yaml")
set(WAYPOINTS_DEFAULT_PATH_ESCAPED "\"${WAYPOINTS_DEFAULT_PATH}\"")
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/config/waypoints_default_path.hpp.in
  ${CMAKE_CURRENT_BINARY_DIR}/generated/waypoints_default_path.hpp
  @ONLY
)


# Add library for waypoint_follower components
add_library(waypoint_follower_lib
  src/waypoint_follower_node.cpp
  src/path_tracker_controller.cpp
  src/odometry_fusion.cpp
  src/waypoint_loader.cpp
  src/trajectory_smoother.cpp
)

# Make generated header available to targets
target_include_directories(waypoint_follower_lib PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/generated)

# Link waypoint_follower_lib with dependencies
target_link_libraries(waypoint_follower_lib
  robotics_control::robotics_control
  yaml-cpp
)

ament_target_dependencies(waypoint_follower_lib
  rclcpp
  rclcpp_action
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  visualization_msgs
  nav2_msgs
  ament_index_cpp
)

# Add executable
add_executable(waypoint_follower_node 
  src/waypoint_follower_node_main.cpp
)

# Target dependencies
ament_target_dependencies(waypoint_follower_node
  rclcpp
  rclcpp_action
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  visualization_msgs
  nav2_msgs
  ament_index_cpp
)

# Link robotics_control library and waypoint_follower_lib
target_link_libraries(waypoint_follower_node
  waypoint_follower_lib
  robotics_control::robotics_control
)

# Installation
install(TARGETS waypoint_follower_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# Install config, launch and rviz assets so runtime can find them via package share
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

ament_package()
