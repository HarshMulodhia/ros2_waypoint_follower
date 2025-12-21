#!/usr/bin/env python3

"""!
@file waypoint_follower.launch.py
@brief ROS 2 launch configuration for complete system
@author Harsh Mulodhia
@version 1.0.0

Launches integrated waypoint following system with:
    - Robot state publisher (URDF transforms)
    - Physics simulator (optional)
    - Waypoint follower node (main controller)
    - RViz visualization (optional)

**Launch Arguments:**

    - use_sim_time (default: false)
        - Enable simulation time for Gazebo/rosbag compatibility

    - use_rviz (default: true)
        - Enable RViz visualization of robot and path

    - use_simulator (default: true)
        - Enable physics_simulator for testing without hardware

    - config_file (default: waypoint_follower_params.yaml)
        - Path to controller parameters file

    - waypoints_file (default: waypoints_course.yaml)
        - Path to waypoints definition file

**TF Hierarchy Established:**
@code
map
└── odom (static, identity transform)
     └── base_link
          ├── wheels
          └── sensors
@endcode

**Node Startup Order:**
1. robot_state_publisher (must be first for URDF)
2. joint_state_publisher
3. static_transform_publisher (map→odom)
4. physics_robot_simulator
5. waypoint_follower_node
6. rviz2

"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch Configuration
    ==================================
    Launches the complete Waypoint Follower system with physics simulation.
    Includes:
    - Robot URDF with all links and joints
    - Physics-based simulator (differential drive kinematics)
    - Static TF publishers (coordinate frame connections)
    - RViz visualization with professional configuration
    - Waypoint follower node for autonomous navigation
    """

    # Get package share directory
    pkg_share = get_package_share_directory('waypoint_follower')

    # Define file paths
    robot_urdf_file = os.path.join(pkg_share, 'config', 'robot.urdf')
    config_file = os.path.join(pkg_share, 'config', 'waypoint_follower_params.yaml')
    waypoints_file = os.path.join(pkg_share, 'config', 'waypoints_course.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'waypoint_follower.rviz')

    # Read robot URDF
    with open(robot_urdf_file, 'r') as f:
        robot_description = f.read()

    # ========== DECLARE LAUNCH ARGUMENTS ==========

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time (for Gazebo compatibility)'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz2 visualization'
    )

    use_simulator_arg = DeclareLaunchArgument(
        'use_simulator',
        default_value='true',
        description='Launch physics simulator'
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=config_file,
        description='Path to waypoint follower config file'
    )

    waypoints_file_arg = DeclareLaunchArgument(
        'waypoints_file',
        default_value=waypoints_file,
        description='Path to waypoints file'
    )

    # ========== NODES ==========

    # 1. CRITICAL: Robot State Publisher (URDF -> TF transforms for wheels & sensors)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    # 2. Static TF Publishers (coordinate frame hierarchy)
    # map -> odom (localization frame connection)
    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_odom',
        output='screen',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--yaw', '0', '--pitch', '0', '--roll', '0',
            '--frame-id', 'map', '--child-frame-id', 'odom'
        ]
    )

    # 3. Physics Simulator (differential drive kinematics)
    # Converts /cmd_vel commands to /odom updates
    physics_simulator = Node(
        package='waypoint_follower',
        executable='physics_simulator.py',
        name='physics_robot_simulator',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_simulator'))
    )

    # 4. Waypoint Follower Node (main control logic)
    waypoint_follower_node = Node(
        package='waypoint_follower',
        executable='waypoint_follower_node',
        name='waypoint_follower_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'waypoints_file': LaunchConfiguration('waypoints_file')}
        ]
    )

    # 5. RViz2 Visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    # ========== BUILD LAUNCH DESCRIPTION ==========

    # Order matters: robot_state_publisher must be first to provide URDF transforms
    # Static TF ensures map frame exists before physics simulator starts
    return LaunchDescription([
        # Launch arguments
        use_sim_time_arg,
        use_rviz_arg,
        use_simulator_arg,
        config_file_arg,
        waypoints_file_arg,

        # Informational messages
        LogInfo(msg="🚀 Launching ROS 2 Waypoint Follower System..."),
        LogInfo(msg="📦 TF Tree: map → odom → base_link → [wheels, sensors]"),

        # Core nodes (in critical order)
        robot_state_publisher,  # CRITICAL: Publishes URDF transforms (base_link → wheels)
        joint_state_publisher,  # Joint state publisher for dynamic transforms
        static_tf_map_odom,     # Ensures map → odom connection exists
        physics_simulator,      # Physics engine (updates odom → base_link)
        waypoint_follower_node, # Path tracking controller
        rviz_node,             # Visualization

        LogInfo(msg="✅ System ready! Check RViz for robot visualization."),
    ])