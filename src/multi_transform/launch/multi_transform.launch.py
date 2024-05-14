from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import LaunchConfiguration 

def generate_launch_description():
    multi_transform_node = Node(
        package='multi_transform',
        executable='multi_transform',
        name='multi_transform',
        output='screen'
    )

    ld = LaunchDescription()

    ld.add_action(multi_transform_node)
    
    return ld