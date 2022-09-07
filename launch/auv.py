from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='msur_driver',
            node_executable='telemetry',
        ),
        Node(
            package='msur_driver',
            node_executable='driver',
        )
    ])
