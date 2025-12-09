import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    package_dir = get_package_share_directory('waypoint_follower')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    rviz_config = LaunchConfiguration('rviz_config',
        default=os.path.join(package_dir, 'rviz', 'waypoint_follower.rviz'))

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    include_waypoint_follower = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_dir, 'launch', 'waypoint_follower.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(include_waypoint_follower)
    ld.add_action(rviz_cmd)

    return ld
