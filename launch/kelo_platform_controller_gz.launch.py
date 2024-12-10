import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    return LaunchDescription([
        # Declare the parameter for platform_max_lin_vel if needed
        DeclareLaunchArgument('platform_max_lin_vel', default_value='1.0', description='Max Linear Velocity'),
        DeclareLaunchArgument('platform_max_ang_vel', default_value='1.0', description='Max Angular Velocity'),

        # Start the KeloGazeboController node
        Node(
            package='kelo_tulip',
            executable='kelo_gazebo_platform_controller',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'platform_max_lin_vel': LaunchConfiguration('platform_max_lin_vel'),
                'platform_max_ang_vel': LaunchConfiguration('platform_max_ang_vel'),
            }],
        ),
    ])