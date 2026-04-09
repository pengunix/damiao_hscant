import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('motor_ros')
    setup_script = os.path.join(pkg_share, 'scripts', 'setup_can.sh')

    return LaunchDescription([
        ExecuteProcess(
            cmd=['bash', setup_script],
            output='screen',
        ),
    ])
