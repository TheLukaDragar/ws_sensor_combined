#!/usr/bin/env python3
"""
Launch file for Agent Auto PX4 mode.
Listens to trajectory commands and controls PX4.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    px4_namespace_arg = DeclareLaunchArgument(
        'px4_namespace',
        default_value='px4_0',
        description='PX4 namespace for multi-vehicle support'
    )
    
    debug_output_arg = DeclareLaunchArgument(
        'debug_output',
        default_value='true',
        description='Enable debug output logging'
    )
    
    trajectory_topic_arg = DeclareLaunchArgument(
        'trajectory_topic',
        default_value='/agent_0/traj_full',
        description='Trajectory topic to subscribe to'
    )
    
    # Create agent_auto node
    agent_auto_node = Node(
        package='agent_auto',
        executable='agent_auto',
        name='agent_auto_node',
        namespace=LaunchConfiguration('px4_namespace'),
        output='screen',
        parameters=[
            {
                'px4_namespace': LaunchConfiguration('px4_namespace'),
                'debug_output': LaunchConfiguration('debug_output'),
                'trajectory_topic': LaunchConfiguration('trajectory_topic'),
            }
        ]
    )
    
    return LaunchDescription([
        px4_namespace_arg,
        debug_output_arg,
        trajectory_topic_arg,
        agent_auto_node
    ])