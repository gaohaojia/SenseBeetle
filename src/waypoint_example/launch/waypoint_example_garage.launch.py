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
            package="waypoint_example",
            executable="waypointExample",
            name="waypointExample",
            output="screen",
            parameters=[{
                'robot_id': LaunchConfiguration('robot_id'),
                "waypoint_file_dir" : os.path.join(get_package_share_directory('waypoint_example'), 'data', 'waypoints_garage.ply'),
                "boundary_file_dir" : os.path.join(get_package_share_directory('waypoint_example'), 'data', 'boundary_garage.ply'),
                "waypointXYRadius" : 0.5,
                "waypointZBound" : 5.0,
                "waitTime" : 0.0,
                "frameRate" : 5.0,
                "speed" : 2.0,
                "sendSpeed" : True,
                "sendBoundary" : True,
            }]
        )
    ])