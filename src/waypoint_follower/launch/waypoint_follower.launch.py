import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    package_dir = get_package_share_directory('waypoint_follower')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    config_file = LaunchConfiguration('config_file',
        default=os.path.join(package_dir, 'config', 'waypoint_follower_params.yaml'))
    waypoints_file = LaunchConfiguration('waypoints_file',
        default=os.path.join(package_dir, 'config', 'waypoints_sample.yaml'))

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file',
        default_value=config_file,
        description='Path to configuration file'
    )

    declare_waypoints_file_cmd = DeclareLaunchArgument(
        'waypoints_file',
        default_value=waypoints_file,
        description='Path to waypoints file'
    )

    waypoint_follower_cmd = Node(
        package='waypoint_follower',
        executable='waypoint_follower_node',
        name='waypoint_follower',
        parameters=[
            config_file,
            {'use_sim_time': use_sim_time},
            {'waypoints_file': waypoints_file}
        ],
        remappings=[
            ('/odom', 'odometry/filtered'),
            ('/imu', 'imu/data'),
            ('/cmd_vel', 'cmd_vel_mux/input/navi')
        ],
        output='screen'
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_config_file_cmd)
    ld.add_action(declare_waypoints_file_cmd)
    ld.add_action(waypoint_follower_cmd)

    return ld
