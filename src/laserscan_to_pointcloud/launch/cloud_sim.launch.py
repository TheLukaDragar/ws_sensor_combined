from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # First, let's add some debug parameters
    debug_level = DeclareLaunchArgument(
        'debug_level',
        default_value='warn',
        description='Debug level: debug, info, warn, error'
    )

    return LaunchDescription([
        debug_level,

        # Connect LiDAR frame to PX4's body frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_px4body2lidar',
            arguments=['0', '0', '0.069', '0', '0', '0', 'px4_body', 'ld19_frame'],
            output='screen',
            parameters=[{
                'qos_overrides./tf_static.publisher.durability': 'transient_local'
            }]
        ),

        # Main node with debug parameters
        Node(
            package='laserscan_to_pointcloud',
            executable='laserscan_to_pointcloud_assembler',
            name='laserscan_to_pointcloud_assembler',
            output='screen',
            arguments=[
                '--ros-args',
                '--log-level', 'info',
                # Silence internal ROS2 debug logs
                '--log-level', 'rcl:=error',
                '--log-level', 'rcutils:=error',
                '--log-level', 'rmw:=error'
            ],
            parameters=[{
                # Debug settings
                'debug_time_sync': True,  # Enable time synchronization debugging
                'debug_tf_lookups': True, # Enable TF lookup debugging
                
                # Core parameters
                'laser_scan_topics': '/ld19_sim/scan',
                'pointcloud_publish_topic': '/ambient_pointcloud',
                'number_of_scans_to_assemble_per_cloud': 20,
                'timeout_for_cloud_assembly': 5.0,  # Increased to allow more scans to accumulate
                
                # Frame configuration - using PX4's frames (case sensitive!)
                'target_frame': 'World',  # PX4's world frame (capital W)
                'motion_estimation_source_frame_id': 'ld19_frame',
                'motion_estimation_target_frame_id': 'World',  # PX4's world frame (capital W)
                'laser_frame': 'ld19_frame',
                
                # PX4 Integration with debug
                'px4_odometry_topic': '/px4_0/fmu/out/vehicle_odometry',
                'px4_attitude_topic': '/px4_0/fmu/out/vehicle_attitude',
                'use_px4_motion_compensation': True,
                'debug_px4_integration': False,  # Disable PX4 integration debugging
                
                # Timing parameters with more lenient settings
                'min_number_of_scans_to_assemble_per_cloud': 1,  # Reduced for debugging
                'max_number_of_scans_to_assemble_per_cloud': 50,
                'min_timeout_seconds_for_cloud_assembly': 2.0,   # Increased minimum timeout
                'max_timeout_seconds_for_cloud_assembly': 10.0,  # Increased maximum timeout
                'tf_lookup_timeout': 1.0,  # Increased for debugging
                
                # Motion parameters
                'max_linear_velocity': 2.0,
                'max_angular_velocity': 1.0,
                
                # Algorithm parameters
                'number_of_tf_queries_for_spherical_interpolation': 4,
                'min_range_cutoff_percentage_offset': 1.05,
                'max_range_cutoff_percentage_offset': 0.95,
                'remove_invalid_measurements': True,
                
                # Additional configuration
                'enforce_reception_of_laser_scans_in_all_topics': False,  # Disabled for initial debugging
                'include_laser_intensity': True,
                
                # Recovery configuration - using PX4's frames
                'recovery_frame': 'World',  # Use World frame for recovery
                'base_link_frame_id': 'body',  # PX4's base frame
                'initial_recovery_transform_in_base_link_to_target': False,  # Don't try to recover, we have direct transforms
                
                # Time sync debugging
                'use_sim_time': True,  # Important for simulation time sync
                'debug_print_timestamps': True,  # Print timestamp information
                
                # QoS Settings
                'qos_overrides./tf_static.subscription.durability': 'transient_local',
                'qos_overrides./tf_static.subscription.reliability': 'reliable'
            }],
            # Add remappings to help with time sync
            remappings=[
                ('/clock', '/simulation/clock'),  # Ensure proper time source
            ],
        )
    ])