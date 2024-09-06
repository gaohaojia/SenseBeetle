from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="loam_interface",
                executable="loamInterface",
                name="loamInterface",
                output="screen",
                parameters=[
                    {
                        "stateEstimationTopic": "Odometry",
                        "registeredScanTopic": "cloud_registered",
                        "flipStateEstimation": False,
                        "flipRegisteredScan": False,
                        "sendTF": True,
                        "reverseTF": False,
                    }
                ],
            )
        ]
    )
