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
            package="terrain_analysis",
            executable="terrainAnalysis",
            name="terrainAnalysis",
            output="screen",
            parameters=[{
                'robot_id': LaunchConfiguration('robot_id'),
                "scanVoxelSize" : 0.05,
                "decayTime" : 2.0,
                "noDecayDis" : 4.0,
                "clearingDis" : 8.0,
                "useSorting" : False,
                "quantileZ" : 0.25,
                "considerDrop" : True,
                "limitGroundLift" : False,
                "maxGroundLift" : 0.15,
                "clearDyObs" : True,
                "minDyObsDis" : 0.3,
                "minDyObsAngle" : 0.0,
                "minDyObsRelZ" : -0.5,
                "absDyObsRelZThre" : 0.2,
                "minDyObsVFOV" : -16.0,
                "maxDyObsVFOV" : 16.0,
                "minDyObsPointNum" : 1,
                "noDataObstacle" : False,
                "noDataBlockSkipNum" : 0,
                "minBlockPointNum" : 10,
                "vehicleHeight" : 1.5,
                "voxelPointUpdateThre" : 100,
                "voxelTimeUpdateThre" : 2.0,
                "minRelZ" : -2.5,
                "maxRelZ" : 1.0,
                "disRatioZ" : 0.2,
            }]
        )
    ])