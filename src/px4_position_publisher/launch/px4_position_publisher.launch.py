from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare the namespace argument
    px4_namespace_arg = DeclareLaunchArgument(
        'px4_namespace',
        default_value='px4_0',
        description='PX4 namespace for multi-vehicle support'
    )

    # Create the node
    position_publisher_node = Node(
        package='px4_position_publisher',
        executable='px4_position_publisher_node',
        name='px4_position_publisher',
        namespace=LaunchConfiguration('px4_namespace'),
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        px4_namespace_arg,
        position_publisher_node
    ])
