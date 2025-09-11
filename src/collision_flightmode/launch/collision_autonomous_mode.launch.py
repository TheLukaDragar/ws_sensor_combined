#!/usr/bin/env python3
"""
Launch file for collision-aware autonomous flight mode.
Integrates with collision prevention node for safe autonomous navigation.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package share directories
    collision_flightmode_share = FindPackageShare('collision_flightmode')
    collision_prevention_share = FindPackageShare('collision_prevention')
    
    # Declare launch arguments
    waypoint_tolerance_arg = DeclareLaunchArgument(
        'waypoint_tolerance',
        default_value='1.0',
        description='Distance tolerance for waypoint arrival (meters)'
    )
    
    max_speed_arg = DeclareLaunchArgument(
        'max_speed',
        default_value='3.0',
        description='Maximum flight speed (m/s)'
    )
    
    collision_distance_threshold_arg = DeclareLaunchArgument(
        'collision_distance_threshold',
        default_value='2.0',
        description='Minimum distance to maintain from obstacles (meters)'
    )
    
    guidance_angle_arg = DeclareLaunchArgument(
        'guidance_angle_deg',
        default_value='30.0',
        description='Maximum angle deviation for obstacle avoidance (degrees)'
    )
    
    debug_output_arg = DeclareLaunchArgument(
        'debug_output',
        default_value='true',
        description='Enable debug output logging'
    )
    
    px4_namespace_arg = DeclareLaunchArgument(
        'px4_namespace',
        default_value='px4_0',
        description='PX4 namespace for multi-vehicle support'
    )
    
    start_collision_prevention_arg = DeclareLaunchArgument(
        'start_collision_prevention',
        default_value='false',
        description='Whether to start the collision prevention node (not needed - using LaserScan directly)'
    )
    
    laser_scan_topic_arg = DeclareLaunchArgument(
        'laser_scan_topic',
        default_value='/ld19_sim/scan',
        description='LaserScan topic for collision detection (fallback)'
    )
    
    use_vdb_mapping_arg = DeclareLaunchArgument(
        'use_vdb_mapping',
        default_value='true',
        description='Enable VDB mapping for 3D collision avoidance'
    )
    
    ambient_pointcloud_topic_arg = DeclareLaunchArgument(
        'ambient_pointcloud_topic',
        default_value='/ambient_pointcloud',
        description='VDB ambient pointcloud topic (spherical interpolation output)'
    )
    
    occupancy_grid_topic_arg = DeclareLaunchArgument(
        'occupancy_grid_topic',
        default_value='/vdb_mapping/vdb_map_occupancy',
        description='VDB occupancy grid topic for persistent mapping'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([collision_flightmode_share, 'config', 'collision_autonomous_mode.yaml']),
        description='Path to collision autonomous mode configuration file'
    )
    
    # Create collision autonomous mode node
    collision_autonomous_mode_node = Node(
        package='collision_flightmode',
        executable='collision_autonomous_mode',
        name='collision_autonomous_mode_node',
        namespace=LaunchConfiguration('px4_namespace'),
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'waypoint_tolerance': LaunchConfiguration('waypoint_tolerance'),
                'max_speed': LaunchConfiguration('max_speed'),
                'collision_distance_threshold': LaunchConfiguration('collision_distance_threshold'),
                'guidance_angle_deg': LaunchConfiguration('guidance_angle_deg'),
                'debug_output': LaunchConfiguration('debug_output'),
                'laser_scan_topic': LaunchConfiguration('laser_scan_topic'),
                'use_vdb_mapping': LaunchConfiguration('use_vdb_mapping'),
                'ambient_pointcloud_topic': LaunchConfiguration('ambient_pointcloud_topic'),
                'occupancy_grid_topic': LaunchConfiguration('occupancy_grid_topic'),
            }
        ]
    )
    
    # Optionally include collision prevention node
    collision_prevention_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                collision_prevention_share,
                'launch',
                'collision_prevention.launch.py'
            ])
        ]),
        condition=IfCondition(LaunchConfiguration('start_collision_prevention'))
    )
    
    # Create waypoint publisher utility (optional)
    waypoint_publisher_node = Node(
        package='collision_flightmode',
        executable='waypoint_publisher',
        name='waypoint_publisher',
        output='screen',
        condition=IfCondition('false')  # Disabled by default
    )
    
    return LaunchDescription([
        waypoint_tolerance_arg,
        max_speed_arg,
        collision_distance_threshold_arg,
        guidance_angle_arg,
        debug_output_arg,
        px4_namespace_arg,
        start_collision_prevention_arg,
        laser_scan_topic_arg,
        use_vdb_mapping_arg,
        ambient_pointcloud_topic_arg,
        occupancy_grid_topic_arg,
        config_file_arg,
        collision_prevention_launch,
        collision_autonomous_mode_node,
        waypoint_publisher_node
    ])