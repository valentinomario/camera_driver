import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

share_dir = get_package_share_directory('camera_driver')
config = os.path.join(share_dir, 'config', 'driver_options.yaml')

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_driver',
            executable='camera_driver_exec',
            name='camera_driver',
            parameters=[config]
        )
    ])
