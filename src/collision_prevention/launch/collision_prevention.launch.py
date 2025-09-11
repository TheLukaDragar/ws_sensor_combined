#!/usr/bin/env python3
"""
Launch file for collision prevention node.
Converts LaserScan data to PX4 ObstacleDistance messages for collision prevention.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package share directory
    pkg_share = FindPackageShare('collision_prevention')
    
    # Declare launch arguments
    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='/ld19_sim/scan',
        description='Input LaserScan topic from the LiDAR sensor'
    )
    
    output_topic_arg = DeclareLaunchArgument(
        'output_topic', 
        default_value='/px4_0/fmu/in/obstacle_distance',
        description='Output ObstacleDistance topic for PX4'
    )
    
    sector_size_arg = DeclareLaunchArgument(
        'sector_size_deg',
        default_value='5',
        description='Sector size in degrees for obstacle distance (PX4 standard is 5 degrees)'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([pkg_share, 'config', 'collision_prevention.yaml']),
        description='Path to collision prevention configuration file'
    )
    
    # PX4 Debug Output Arguments
    obstacle_input_topic_arg = DeclareLaunchArgument(
        'px4_obstacle_topic',
        default_value='/px4_0/fmu/out/obstacle_distance_fused',
        description='PX4 ObstacleDistance topic to convert back to LaserScan for debugging'
    )
    
    debug_laser_output_arg = DeclareLaunchArgument(
        'debug_laser_topic',
        default_value='/px4_debug_fused_obstacle_distance_laser_scan',
        description='Debug LaserScan topic showing PX4 processed obstacle data'
    )
    
    debug_frame_id_arg = DeclareLaunchArgument(
        'debug_frame_id',
        default_value='base_link',
        description='Frame ID for the debug laser scan (should match sensor frame)'
    )
    
    # Create collision prevention node
    collision_prevention_node = Node(
        package='collision_prevention',
        executable='collision_prevention_node',
        name='collision_prevention_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'input_topic': LaunchConfiguration('input_topic'),
                'output_topic': LaunchConfiguration('output_topic'), 
                'sector_size_deg': LaunchConfiguration('sector_size_deg'),
            }
        ],
        remappings=[
            # Add any topic remappings here if needed
        ]
    )
    
    # Create PX4 debug node (converts PX4 ObstacleDistance back to LaserScan for visualization)
    px4_debug_laser_node = Node(
        package='collision_prevention',
        executable='obstacle_to_laser_node',
        name='px4_debug_laser_node',
        output='screen',
        parameters=[
            {
                'input_topic': LaunchConfiguration('px4_obstacle_topic'),
                'output_topic': LaunchConfiguration('debug_laser_topic'),
                'frame_id': LaunchConfiguration('debug_frame_id'),
                'debug_output': True
            }
        ],
        remappings=[
            # Add any topic remappings here if needed
        ]
    )
    
    return LaunchDescription([
        input_topic_arg,
        output_topic_arg,
        sector_size_arg,
        config_file_arg,
        obstacle_input_topic_arg,
        debug_laser_output_arg,
        debug_frame_id_arg,
        collision_prevention_node,
        px4_debug_laser_node
    ])