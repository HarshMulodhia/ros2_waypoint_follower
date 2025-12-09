# Library sources
set(CONTROL_SOURCES
    pid_controller.cpp
    lqr_controller.cpp
    state_space.cpp
    kalman_filter.cpp
    transform.cpp
)

# Create library
add_library(robotics_control STATIC ${CONTROL_SOURCES})

# Link Eigen
target_link_libraries(robotics_control PUBLIC Eigen3::Eigen)

# Include directories
# Make headers available at both paths for compatibility
target_include_directories(robotics_control
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include/robotics_control>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
    $<INSTALL_INTERFACE:include>
)

# Install library headers in robotics_control namespace
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../include/robotics_control/
    DESTINATION include/robotics_control
)

# Also install headers directly in include for compatibility
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../include/robotics_control/control
    DESTINATION include
)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../include/robotics_control/estimation
    DESTINATION include
)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../include/robotics_control/maths
    DESTINATION include
)

# Install library
install(TARGETS robotics_control
    EXPORT robotics_control_targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(EXPORT robotics_control_targets
    FILE robotics_control_targets.cmake
    NAMESPACE robotics_control::
    DESTINATION lib/cmake/robotics_control
)


