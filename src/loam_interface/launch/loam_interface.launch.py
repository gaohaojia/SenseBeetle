import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="loam_interface",
            executable="loamInterface",
            name="loamInterface",
            output="screen",
            parameters=[{
                'robot_id': LaunchConfiguration('robot_id'),
                "stateEstimationTopic" : "Odometry",
                "registeredScanTopic" : "cloud_registered",
                "flipStateEstimation" : False,
                "flipRegisteredScan" : False,
                "sendTF" : True,
                "reverseTF" : False,
            }]
        )
    ])