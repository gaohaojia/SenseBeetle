from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="sensor_scan_generation",
                executable="sensorScanGeneration",
                name="sensorScanGeneration",
                output="screen",
            )
        ]
    )
