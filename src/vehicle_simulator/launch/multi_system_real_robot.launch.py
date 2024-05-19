import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, OpaqueFunction, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import LaunchConfiguration
from launch.conditions import LaunchConfigurationEquals

def push_namespace(context: LaunchContext, robot_id):
    id_str = context.perform_substitution(robot_id)
    return [PushRosNamespace('robot_' + str(id_str))]

def launch_rviz_node(context: LaunchContext, robot_id):
    id_str = context.perform_substitution(robot_id)
    rviz_config_file = os.path.join(get_package_share_directory('vehicle_simulator'), 'rviz', 'multi_' + id_str + '.rviz')
    start_rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )
    delayed_start_rviz = TimerAction(
        period=8.0,
        actions=[start_rviz]
    )
    return [delayed_start_rviz]

def generate_launch_description():
    robot_id = LaunchConfiguration('robot_id')
    lio_mode = LaunchConfiguration('lio_mode')
    cameraOffsetZ = LaunchConfiguration('cameraOffsetZ')
    vehicleX = LaunchConfiguration('vehicleX')
    vehicleY = LaunchConfiguration('vehicleY')
    checkTerrainConn = LaunchConfiguration('checkTerrainConn')
    
    declare_robot_id = DeclareLaunchArgument('robot_id', default_value='0', description='')
    declare_lio_mode = DeclareLaunchArgument('lio_mode', default_value='point_lio', description='')
    declare_cameraOffsetZ = DeclareLaunchArgument('cameraOffsetZ', default_value='0.0', description='')
    declare_vehicleX = DeclareLaunchArgument('vehicleX', default_value='0.0', description='')
    declare_vehicleY = DeclareLaunchArgument('vehicleY', default_value='0.0', description='')
    declare_checkTerrainConn = DeclareLaunchArgument('checkTerrainConn', default_value='true', description='')

    start_livox_mid360 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('livox_ros_driver2'), 'launch', 'msg_MID360_launch.py')
        )
    )

    start_fast_lio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('fast_lio'), 'launch', 'mapping_mid360.launch.py')
        ),
        launch_arguments={
            'robot_id': robot_id,
        }.items(),
        condition=LaunchConfigurationEquals('lio_mode', 'fast_lio')
    )

    start_point_lio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('point_lio'), 'launch', 'mapping_mid360.launch.py')
        ),
        launch_arguments={
            'robot_id': robot_id,
        }.items(),
        condition=LaunchConfigurationEquals('lio_mode', 'point_lio')
    )

    start_multi_transform = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('multi_transform'), 'launch', 'multi_transform.launch.py')
        ),
        launch_arguments={
            'robot_id': robot_id
        }.items()
    )
    
    start_local_planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('local_planner'), 'launch', 'local_planner.launch.py')
        ),
        launch_arguments={
            'robot_id': robot_id,
            'cameraOffsetZ': cameraOffsetZ,
            'goalX': vehicleX,
            'goalY': vehicleY,
        }.items()
    )

    start_terrain_analysis = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('terrain_analysis'), 'launch', 'terrain_analysis.launch.py')
        ),
        launch_arguments={
            'robot_id': robot_id,
        }.items()
    )

    start_terrain_analysis_ext = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('terrain_analysis_ext'), 'launch', 'terrain_analysis_ext.launch.py')
        ),
        launch_arguments={
            'robot_id': robot_id,
            'checkTerrainConn': checkTerrainConn,
        }.items()
    )

    start_sensor_scan_generation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('sensor_scan_generation'), 'launch', 'sensor_scan_generation.launch.py')
        ),
        launch_arguments={
            'robot_id': robot_id,
        }.items()
    )

    start_loam_interface = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('loam_interface'), 'launch', 'loam_interface.launch.py')
        ),
        launch_arguments={
            'robot_id': robot_id,
        }.items()
    )

    start_joy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('joy'), 'launch', 'joy-launch.py')
        )
    )

    ld = LaunchDescription()

    # Add the actions
    ld.add_action(declare_robot_id)
    ld.add_action(declare_lio_mode)
    ld.add_action(declare_cameraOffsetZ)
    ld.add_action(declare_vehicleX)
    ld.add_action(declare_vehicleY)
    ld.add_action(declare_checkTerrainConn)

    ld.add_action(OpaqueFunction(function=push_namespace, args=[robot_id]))

    ld.add_action(start_livox_mid360)
    ld.add_action(start_fast_lio)
    ld.add_action(start_point_lio)
    ld.add_action(TimerAction(period=10.0, actions=[start_multi_transform]))
    ld.add_action(start_local_planner)
    ld.add_action(start_terrain_analysis)
    ld.add_action(start_terrain_analysis_ext)
    ld.add_action(start_sensor_scan_generation)
    ld.add_action(start_loam_interface)
    ld.add_action(start_joy)
    ld.add_action(OpaqueFunction(function=launch_rviz_node, args=[robot_id]))

    return ld