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
            # remappings=[
            #     ('/tf', 'tf'),
            #     ('/tf_static', 'tf_static'),
            # ],
            parameters=[{
                "stateEstimationTopic" : "Odometry",
                "registeredScanTopic" : "cloud_registered",
                "flipStateEstimation" : True,
                "flipRegisteredScan" : True,
                "sendTF" : True,
                "reverseTF" : False,
            }]
        )
    ])