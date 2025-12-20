import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('waypoint_follower')
    
    # File paths
    urdf_file = os.path.join(pkg_share, 'config', 'robot.urdf')
    world_file = os.path.join(pkg_share, 'config', 'warehouse.world')
    rviz_config = os.path.join(pkg_share, 'rviz', 'waypoint_follower.rviz')
    params_file = os.path.join(pkg_share, 'config', 'waypoint_follower_params.yaml')
    
    # Read URDF
    with open(urdf_file, 'r') as f:
        robot_description = f.read()
    
    # Gazebo server
    gazebo_server = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_file],
        output='screen'
    )
    
    # Gazebo client (GUI)
    gazebo_client = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )
    
    # Gazebo-ROS bridge (spawns robot + publishes tf)
    spawn_robot = ExecuteProcess(
        cmd=['spawn_entity.py', '-topic', '/robot_description', '-entity', 'waypoint_follower'],
        output='screen'
    )
    
    # Waypoint follower node
    waypoint_follower = Node(
        package='waypoint_follower',
        executable='waypoint_follower_node',
        name='waypoint_follower_node',
        output='screen',
        parameters=[params_file]
    )
    
    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )
    
    return LaunchDescription([
        gazebo_server,
        gazebo_client,
        robot_state_publisher,
        spawn_robot,
        waypoint_follower,
        rviz_node
    ])
