from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import TextSubstitution

def generate_launch_description():
    # Declare all launch arguments
    laser_scan_topics = DeclareLaunchArgument(
        'laser_scan_topics',
        default_value='/scan',
        description='Several input sensor_msgs/LaserScan topics can be specified using the + separator'
    )

    pointcloud_publish_topic = DeclareLaunchArgument(
        'pointcloud_publish_topic',
        default_value='scan_pointcloud',
        description='Topic name for the assembled point cloud'
    )

    enforce_reception = DeclareLaunchArgument(
        'enforce_reception_of_laser_scans_in_all_topics',
        default_value='true',
        description='Wait for scans from all topics before publishing'
    )

    include_intensity = DeclareLaunchArgument(
        'include_laser_intensity',
        default_value='true',
        description='Include intensity field in point cloud'
    )

    scans_per_cloud = DeclareLaunchArgument(
        'number_of_scans_to_assemble_per_cloud',
        default_value='1',
        description='Number of scans to assemble before publishing'
    )

    timeout_assembly = DeclareLaunchArgument(
        'timeout_for_cloud_assembly',
        default_value='1.0',
        description='Timeout in seconds for cloud assembly'
    )

    # Motion estimation parameters
    target_frame = DeclareLaunchArgument(
        'target_frame',
        default_value='local_origin',
        description='Target frame for point cloud assembly'
    )

    motion_source = DeclareLaunchArgument(
        'motion_estimation_source_frame_id',
        default_value='base_laser',
        description='Source frame for motion estimation'
    )

    motion_target = DeclareLaunchArgument(
        'motion_estimation_target_frame_id',
        default_value='local_origin',
        description='Target frame for motion estimation'
    )

    # Static transform for laser
    static_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_fcu2laser',
        arguments=['0', '0', '0.042', '0', '0', '0', 'fcu', 'base_laser']
    )

    # Main node
    laserscan_to_pointcloud = Node(
        package='laserscan_to_pointcloud',
        executable='laserscan_to_pointcloud_assembler',
        name='laserscan_to_pointcloud_assembler',
        parameters=[{
            'laser_scan_topics': LaunchConfiguration('laser_scan_topics'),
            'pointcloud_publish_topic': LaunchConfiguration('pointcloud_publish_topic'),
            'enforce_reception_of_laser_scans_in_all_topics': LaunchConfiguration('enforce_reception_of_laser_scans_in_all_topics'),
            'include_laser_intensity': LaunchConfiguration('include_laser_intensity'),
            'number_of_scans_to_assemble_per_cloud': LaunchConfiguration('number_of_scans_to_assemble_per_cloud'),
            'timeout_for_cloud_assembly': LaunchConfiguration('timeout_for_cloud_assembly'),
            'target_frame': LaunchConfiguration('target_frame'),
            'motion_estimation_source_frame_id': LaunchConfiguration('motion_estimation_source_frame_id'),
            'motion_estimation_target_frame_id': LaunchConfiguration('motion_estimation_target_frame_id'),
            
            # Additional parameters with default values
            'min_number_of_scans_to_assemble_per_cloud': 1,
            'max_number_of_scans_to_assemble_per_cloud': 10,
            'min_timeout_seconds_for_cloud_assembly': 0.3,
            'max_timeout_seconds_for_cloud_assembly': 1.3,
            'max_linear_velocity': 0.035,
            'max_angular_velocity': 0.1,
            'laser_frame': '',
            'number_of_tf_queries_for_spherical_interpolation': 4,
            'tf_lookup_timeout': 0.15,
            'min_range_cutoff_percentage_offset': 2.00,
            'max_range_cutoff_percentage_offset': 0.95,
            'remove_invalid_measurements': True,
            'recovery_frame': 'local_origin',
            'base_link_frame_id': 'base_laser',
            'initial_recovery_transform_in_base_link_to_target': False,
            'recovery_to_target_frame_transform_initial_x': 0.0,
            'recovery_to_target_frame_transform_initial_y': 0.0,
            'recovery_to_target_frame_transform_initial_z': 0.0,
            'recovery_to_target_frame_transform_initial_roll': 0.0,
            'recovery_to_target_frame_transform_initial_pitch': 0.0,
            'recovery_to_target_frame_transform_initial_yaw': 0.0,
            
            # PX4 specific parameters
            'px4_odometry_topic': '/px4_0/fmu/out/vehicle_odometry',
            'px4_attitude_topic': '/px4_0/fmu/out/vehicle_attitude',
            'use_px4_motion_compensation': True,
        }],
        output='screen'
    )

    return LaunchDescription([
        # Launch arguments
        laser_scan_topics,
        pointcloud_publish_topic,
        enforce_reception,
        include_intensity,
        scans_per_cloud,
        timeout_assembly,
        target_frame,
        motion_source,
        motion_target,
        
        # Nodes
        static_transform,
        laserscan_to_pointcloud
    ])
