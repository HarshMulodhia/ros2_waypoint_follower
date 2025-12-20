#!/usr/bin/env python3

"""
Waypoint Follower ROS2 Launch File
====================================

This launch file starts the complete waypoint follower system with:
- Waypoint follower node
- Parameter server
- Optional RViz visualization
- Optional simulation

Usage:
    ros2 launch waypoint_follower waypoint_follower.launch.py
    ros2 launch waypoint_follower waypoint_follower.launch.py use_rviz:=true
    ros2 launch waypoint_follower waypoint_follower.launch.py use_sim_time:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    """Generate the launch description for waypoint follower."""
    
    # Get package paths
    pkg_share = FindPackageShare('waypoint_follower')
    config_dir = PathJoinSubstitution([pkg_share, 'config'])
    rviz_config = PathJoinSubstitution([pkg_share, 'rviz', 'waypoint_follower.rviz'])
    
    # Declare launch arguments
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Launch RViz visualization'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    waypoints_file_arg = DeclareLaunchArgument(
        'waypoints_file',
        default_value=os.path.join(config_dir, 'waypoints_sample.yaml'),
        description='Path to waypoints YAML file'
    )
    
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(config_dir, 'waypoint_follower_params.yaml'),
        description='Path to parameters YAML file'
    )
    
    # ========================================================================
    # Waypoint Follower Node
    # ========================================================================
    
    waypoint_follower_node = Node(
        package='waypoint_follower',
        executable='waypoint_follower_node',
        name='waypoint_follower_node',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'waypoints_file': LaunchConfiguration('waypoints_file'),
            }
        ],
        remappings=[
            ('/odom', '/odometry/filtered'),
            ('/imu/data', '/imu'),
            ('/cmd_vel', '/cmd_vel'),
        ]
    )
    
    # ========================================================================
    # RViz Visualization (Optional)
    # ========================================================================
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        output='screen'
    )
    
    # ========================================================================
    # Launch Description
    # ========================================================================
    
    ld = LaunchDescription([
        use_rviz_arg,
        use_sim_time_arg,
        waypoints_file_arg,
        params_file_arg,
        waypoint_follower_node,
        rviz_node,
    ])
    
    return ld
