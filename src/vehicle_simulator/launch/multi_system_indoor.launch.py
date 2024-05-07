import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import LaunchConfiguration 

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
        actions=[
        start_rviz
        ]
    )
    return [delayed_start_rviz]

def generate_launch_description():
    robot_id = LaunchConfiguration('robot_id')
    world_name = LaunchConfiguration('world_name')
    vehicleHeight = LaunchConfiguration('vehicleHeight')
    cameraOffsetZ = LaunchConfiguration('cameraOffsetZ')
    vehicleX = LaunchConfiguration('vehicleX')
    vehicleY = LaunchConfiguration('vehicleY')
    vehicleZ = LaunchConfiguration('vehicleZ')
    terrainZ = LaunchConfiguration('terrainZ')
    vehicleYaw = LaunchConfiguration('vehicleYaw')
    gazebo_gui = LaunchConfiguration('gazebo_gui')
    checkTerrainConn = LaunchConfiguration('checkTerrainConn')
    
    declare_world_name = DeclareLaunchArgument('world_name', default_value='indoor', description='')
    declare_vehicleHeight = DeclareLaunchArgument('vehicleHeight', default_value='0.75', description='')
    declare_cameraOffsetZ = DeclareLaunchArgument('cameraOffsetZ', default_value='0.0', description='')
    declare_vehicleX = DeclareLaunchArgument('vehicleX', default_value='0.0', description='')
    declare_vehicleY = DeclareLaunchArgument('vehicleY', default_value='0.0', description='')
    declare_vehicleZ = DeclareLaunchArgument('vehicleZ', default_value='0.0', description='')
    declare_terrainZ = DeclareLaunchArgument('terrainZ', default_value='0.0', description='')
    declare_vehicleYaw = DeclareLaunchArgument('vehicleYaw', default_value='0.0', description='')
    declare_gazebo_gui = DeclareLaunchArgument('gazebo_gui', default_value='false', description='')
    declare_checkTerrainConn = DeclareLaunchArgument('checkTerrainConn', default_value='true', description='')
    declare_robot_id = DeclareLaunchArgument('robot_id', default_value='0', description='')
    
    start_local_planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('local_planner'), 'launch', 'local_planner.launch.py')
        ),
        launch_arguments={
            'cameraOffsetZ': cameraOffsetZ,
            'goalX': vehicleX,
            'goalY': vehicleY,
        }.items()
    )

    start_terrain_analysis = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('terrain_analysis'), 'launch', 'terrain_analysis.launch.py')
        )
    )

    start_terrain_analysis_ext = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('terrain_analysis_ext'), 'launch', 'terrain_analysis_ext.launch.py')
        ),
        launch_arguments={
            'checkTerrainConn': checkTerrainConn,
            'robot_id': robot_id
        }.items()
    )

    start_vehicle_simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('vehicle_simulator'), 'launch', 'vehicle_simulator.launch.py')
        ),
        launch_arguments={
            'world_name': world_name,
            'vehicleHeight': vehicleHeight,
            'cameraOffsetZ': cameraOffsetZ,
            'vehicleX': vehicleX,
            'vehicleY': vehicleY,
            'terrainZ': terrainZ,
            'vehicleYaw': vehicleYaw,
            'gui': gazebo_gui,
        }.items()
    )

    start_sensor_scan_generation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('sensor_scan_generation'), 'launch', 'sensor_scan_generation.launch.py')
        )
    )

    start_visualization_tools = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(os.path.join(
            get_package_share_directory('visualization_tools'), 'launch', 'visualization_tools.launch')
        ),
        launch_arguments={
            'world_name': world_name,
        }.items()
    )

    ld = LaunchDescription()

    # Add the actions
    ld.add_action(declare_robot_id)
    ld.add_action(declare_world_name)
    ld.add_action(declare_vehicleHeight)
    ld.add_action(declare_cameraOffsetZ)
    ld.add_action(declare_vehicleX)
    ld.add_action(declare_vehicleY)
    ld.add_action(declare_vehicleZ)
    ld.add_action(declare_terrainZ)
    ld.add_action(declare_vehicleYaw)
    ld.add_action(declare_gazebo_gui)
    ld.add_action(declare_checkTerrainConn)

    ld.add_action(OpaqueFunction(function=push_namespace, args=[robot_id]))

    ld.add_action(start_local_planner)
    ld.add_action(start_terrain_analysis)
    ld.add_action(start_terrain_analysis_ext)
    ld.add_action(start_vehicle_simulator)
    ld.add_action(start_sensor_scan_generation)
    ld.add_action(start_visualization_tools)
    ld.add_action(OpaqueFunction(function=launch_rviz_node, args=[robot_id]))

    return ld