import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import LaunchConfiguration

def visualization_tools_node(context: LaunchContext, world_name):
    world_name_str = context.perform_substitution(world_name)
    return [Node(
        package='visualization_tools',
        executable='visualizationTools',
        name='visualizationTools',
        output='screen',
        parameters=[{
            "metricFile" : os.path.join(get_package_share_directory('vehicle_simulator'), 'log', 'metrics'),
            "trajFile" : os.path.join(get_package_share_directory('vehicle_simulator'), 'log', 'trajectory'),
            "mapFile" : os.path.join(get_package_share_directory('vehicle_simulator'), 'mesh', world_name_str, 'preview', 'pointcloud.ply'),
            "overallMapVoxelSize" : 0.5,
            "exploredAreaVoxelSize" : 0.3,
            "exploredVolumeVoxelSize" : 0.5,
            "transInterval" : 0.2,
            "yawInterval" : 10.0,
            "overallMapDisplayInterval" : 2,
            "exploredAreaDisplayInterval" : 1,
        }]
    )]

def generate_launch_description():
    world_name = LaunchConfiguration('world_name')
    declare_world_name = DeclareLaunchArgument('world_name', default_value='garage', description='')

    real_time_plot_node = Node(
        package='visualization_tools',
        executable='realTimePlot.py',
        name='realTimePlot',
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(declare_world_name)
    ld.add_action(OpaqueFunction(function=visualization_tools_node, args=[world_name]))
    ld.add_action(real_time_plot_node)
    return ld
