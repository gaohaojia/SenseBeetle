from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    robot_id = LaunchConfiguration('robot_id')
    declare_robot_id = DeclareLaunchArgument('robot_id', default_value='0', description='')

    small_gicp_node = Node(
        package='explored_area',
        executable='explored_area_node',
        name='explored_area',
        output='screen',
        parameters=[{
            'robot_id': robot_id
        }]
    )

    ld = LaunchDescription()
    ld.add_action(declare_robot_id)

    ld.add_action(small_gicp_node)
    
    return ld