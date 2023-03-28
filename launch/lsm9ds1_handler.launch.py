import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('lsm9ds1_handler'),
        'config',
        'params.yaml'
        )
    return LaunchDescription([
        Node(
            name='lsm9ds1_handler',
            package='lsm9ds1_handler',
            executable='lsm9ds1_node',
            output="screen",
            emulate_tty=True,
            parameters=[config]
        )
    ])
