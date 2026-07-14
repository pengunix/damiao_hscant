import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('motor_ros')
    default_config = os.path.join(pkg_share, 'conf', 'motors.yaml')

    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to the motor configuration YAML file'
    )

    motor_node = Node(
        package='motor_ros',
        executable='motor_node',
        name='motor_ros',
        output='screen',
        arguments=['--config', LaunchConfiguration('config_file')],
    )

    return LaunchDescription([
        config_arg,
        motor_node,
    ])
