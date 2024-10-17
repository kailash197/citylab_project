import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
    return LaunchDescription([
        # Declare an argument for the RViz configuration file path
        DeclareLaunchArgument(
            'rviz_config',
            # Always provide full path.
            default_value=PathJoinSubstitution([os.environ['HOME'], 'ros2_ws/src/citylab_project/robot_patrol/rviz/robot_environment_config.rviz']),  
            description='Path to the RViz configuration file'
        ),
        
        # Launch the robot patrol node
        Node(
            package='robot_patrol',
            executable='robot_patrol_node',
            output='screen'
        ),

        # Launch RViz with the specified configuration
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config')]
        ),
    ])
