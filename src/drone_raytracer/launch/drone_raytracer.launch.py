#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'drone_frame',
            default_value='px4_body',
            description='Frame ID of the drone'
        ),
        DeclareLaunchArgument(
            'world_frame',
            default_value='local_origin',
            description='Frame ID of the world/map'
        ),
        DeclareLaunchArgument(
            'ray_length',
            default_value='10.0',
            description='Maximum length of rays in meters'
        ),
        DeclareLaunchArgument(
            'raytracing_rate',
            default_value='5.0',
            description='Rate of raytracing in Hz'
        ),
        DeclareLaunchArgument(
            'num_rays_horizontal',
            default_value='16',
            description='Number of rays in horizontal direction'
        ),
        DeclareLaunchArgument(
            'num_rays_vertical',
            default_value='8',
            description='Number of rays in vertical direction'
        ),
        DeclareLaunchArgument(
            'horizontal_fov',
            default_value='6.283185',  # 2*pi
            description='Horizontal field of view in radians'
        ),
        DeclareLaunchArgument(
            'vertical_fov',
            default_value='3.141593',  # pi
            description='Vertical field of view in radians'
        ),
        
        Node(
            package='drone_raytracer',
            executable='drone_raytracer_node',
            name='drone_raytracer_node',
            output='screen',
            parameters=[{
                'drone_frame': LaunchConfiguration('drone_frame'),
                'world_frame': LaunchConfiguration('world_frame'),
                'ray_length': LaunchConfiguration('ray_length'),
                'raytracing_rate': LaunchConfiguration('raytracing_rate'),
                'num_rays_horizontal': LaunchConfiguration('num_rays_horizontal'),
                'num_rays_vertical': LaunchConfiguration('num_rays_vertical'),
                'horizontal_fov': LaunchConfiguration('horizontal_fov'),
                'vertical_fov': LaunchConfiguration('vertical_fov'),
            }]
        )
    ])