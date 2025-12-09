cmake_minimum_required(VERSION 3.14)
project(robotics_control VERSION 1.0.0 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic")

# Find ament_cmake for ROS2
find_package(ament_cmake REQUIRED)

# Find Eigen
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Add include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(/usr/include/eigen3)  # For Eigen

# Add source subdirectory
add_subdirectory(src)

# Export CMake targets for other packages to find this library
ament_export_targets(robotics_control_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)

# Register the package
ament_package()

# Add tests (optional, only if tests directory exists)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests")
  enable_testing()
  add_subdirectory(tests)
endif()
