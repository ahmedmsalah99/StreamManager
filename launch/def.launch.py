from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Locate the YAML configuration file
    package_share = FindPackageShare('stream_manager').find('stream_manager')
    config_file = os.path.join(package_share, 'config', 'stream_manager.yaml')

    # Define the node
    stream_manager_node = Node(
        package='stream_manager',
        executable='stream_manager_node',
        name='stream_manager',
        output='screen',
        parameters=[config_file],
    )

    return LaunchDescription([stream_manager_node])
