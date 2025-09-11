from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Launch arguments
    laser_scan_topic = DeclareLaunchArgument(
        'laser_scan_topic',
        default_value='/ld19_sim/scan',
        description='Laser scan input topic'
    )
    
    pointcloud_topic = DeclareLaunchArgument(
        'pointcloud_topic', 
        default_value='/ambient_pointcloud',
        description='Output pointcloud topic'
    )
    
    num_scans = DeclareLaunchArgument(
        'num_scans',
        default_value='20',
        description='Number of scans to assemble per cloud'
    )

    return LaunchDescription([
        laser_scan_topic,
        pointcloud_topic,
        num_scans,

        # Static transforms now handled by PX4 TF Publisher

        # Main point cloud assembler with PX4 direct integration
        Node(
            package='laserscan_to_pointcloud',
            executable='laserscan_to_pointcloud_assembler',
            name='px4_pointcloud_assembler',
            output='screen',
            arguments=[
                '--ros-args',
                '--log-level', 'info'
            ],
            parameters=[{
                # === Core Parameters ===
                'laser_scan_topics': LaunchConfiguration('laser_scan_topic'),
                'pointcloud_publish_topic': LaunchConfiguration('pointcloud_topic'),
                'number_of_scans_to_assemble_per_cloud': LaunchConfiguration('num_scans'),
                'timeout_for_cloud_assembly': 0.5,
                
                # === Frame Configuration ===
                # Use local_origin for hardware-independent operation
                'target_frame': 'local_origin',  # PX4's local origin frame
                'laser_frame': 'ld19_frame',  # Laser sensor frame
                
                # Motion estimation not needed - using PX4 direct compensation
                'motion_estimation_source_frame_id': '',
                'motion_estimation_target_frame_id': '',
                
                # === PX4 Direct Motion Compensation ===
                'use_px4_motion_compensation': True,  # Enable PX4 direct integration
                'px4_odometry_topic': '/px4_0/fmu/out/vehicle_odometry',
                'px4_attitude_topic': '/px4_0/fmu/out/vehicle_attitude',
                'debug_px4_integration': True,  # Enable for initial testing
                
                # === Timing Parameters ===
                'min_number_of_scans_to_assemble_per_cloud': 3,
                'max_number_of_scans_to_assemble_per_cloud': 50,
                'min_timeout_seconds_for_cloud_assembly': 1.0,
                'max_timeout_seconds_for_cloud_assembly': 8.0,
                'tf_lookup_timeout': 0.2,
                
                # === Motion Parameters ===
                'max_linear_velocity': 10.0,  # m/s - adjust for your vehicle
                'max_angular_velocity': 2.0,  # rad/s - adjust for your vehicle
                
                # === Algorithm Parameters ===
                'number_of_tf_queries_for_spherical_interpolation': 32,
                'min_range_cutoff_percentage_offset': 1.05,
                'max_range_cutoff_percentage_offset': 0.95,
                'remove_invalid_measurements': True,
                
                # === Additional Configuration ===
                'enforce_reception_of_laser_scans_in_all_topics': False,
                'include_laser_intensity': True,
                
                # === Recovery Configuration ===
                'recovery_frame': 'local_origin',  # Same as target frame
                'base_link_frame_id': 'px4_body',  # PX4's body frame
                'initial_recovery_transform_in_base_link_to_target': False,
                
                # === Time Synchronization ===
                'use_sim_time': False,  # Set to false for real hardware
                'debug_time_sync': True,  # Enable for debugging
                'debug_tf_lookups': False,  # Disable TF debugging since we're not using TF motion estimation
                'debug_print_timestamps': True,
                
                # === QoS Settings for PX4 compatibility ===
                'qos_overrides./tf_static.subscription.durability': 'transient_local',
                'qos_overrides./tf_static.subscription.reliability': 'reliable'
            }]
        )
    ])