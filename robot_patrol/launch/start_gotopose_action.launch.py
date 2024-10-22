import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, DeclareLaunchArgument
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

        # Launch RViz with the specified configuration
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config')]
        ),
        
        # Start patrol node after a delay
        TimerAction(
            period=1.0,  #delay
            actions=[
                Node(
                    package='robot_patrol',
                    executable='gotopose_service_node',
                    output='screen'
                )
            ]
        )
    ])
