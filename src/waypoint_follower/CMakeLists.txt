cmake_minimum_required(VERSION 3.8)
project(waypoint_follower)

# C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(robotics_control REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Include directories
include_directories(include)

# Define dependencies
set(WAYPOINT_FOLLOWER_DEPS
    rclcpp
    rclcpp_action
    nav2_msgs
    nav_msgs
    sensor_msgs
    geometry_msgs
    std_msgs
    robotics_control
)

# Build library
add_library(waypoint_follower_lib SHARED
    src/waypoint_loader.cpp
    src/path_tracker_controller.cpp
    src/odometry_fusion.cpp
    src/trajectory_smoother.cpp
)

target_include_directories(waypoint_follower_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(waypoint_follower_lib ${WAYPOINT_FOLLOWER_DEPS})
target_link_libraries(waypoint_follower_lib yaml-cpp Eigen3::Eigen)

# Build executable
add_executable(waypoint_follower_node
    src/waypoint_follower_node.cpp
    src/waypoint_follower_node_main.cpp
)

target_include_directories(waypoint_follower_node PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

ament_target_dependencies(waypoint_follower_node ${WAYPOINT_FOLLOWER_DEPS})
target_link_libraries(waypoint_follower_node waypoint_follower_lib yaml-cpp Eigen3::Eigen)

# Install library
install(TARGETS waypoint_follower_lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# Install executable
install(TARGETS waypoint_follower_node
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
    DESTINATION include
)

# Make Python scripts executable and install them
install(PROGRAMS
    scripts/professional_simulator.py
    DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
)

# Install configuration files
install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
)

# Install rviz config
install(DIRECTORY rviz/
    DESTINATION share/${PROJECT_NAME}/rviz
)

# Testing
if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_waypoint_loader test/test_waypoint_loader.cpp)
    target_link_libraries(test_waypoint_loader waypoint_follower_lib)
    ament_add_gtest(test_path_tracker test/test_path_tracker.cpp)
    target_link_libraries(test_path_tracker waypoint_follower_lib)
    ament_add_gtest(test_odometry_fusion test/test_odometry_fusion.cpp)
    target_link_libraries(test_odometry_fusion waypoint_follower_lib)
    ament_add_gtest(test_trajectory_smoother test/test_trajectory_smoother.cpp)
    target_link_libraries(test_trajectory_smoother waypoint_follower_lib)
endif()

ament_package()
