from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import LaunchConfiguration 

def generate_launch_description():
    robot_id = LaunchConfiguration('robot_id')
    checkTerrainConn = LaunchConfiguration('checkTerrainConn')
    declare_robot_id = DeclareLaunchArgument('robot_id', default_value='0', description='')
    declare_checkTerrainConn = DeclareLaunchArgument('checkTerrainConn', default_value='true', description='')

    terrain_analysis_ext_node = Node(
        package='terrain_analysis_ext',
        executable='terrainAnalysisExt',
        name='terrainAnalysisExt',
        output='screen',
        parameters=[{
            'robot_id': robot_id,
            "scanVoxelSize": 0.1,
            "decayTime": 10.0,
            "noDecayDis": 0.0,
            "clearingDis": 30.0,
            "useSorting": False,
            "quantileZ": 0.1,
            "vehicleHeight": 1.5,
            "voxelPointUpdateThre": 100,
            "voxelTimeUpdateThre": 2.0,
            "lowerBoundZ": -2.5,
            "upperBoundZ": 1.0,
            "disRatioZ": 0.1,
            "checkTerrainConn": checkTerrainConn,
            "terrainConnThre": 0.5,
            "terrainUnderVehicle": -0.75,
            "ceilingFilteringThre": 2.0,
            "localTerrainMapRadius": 4.0
        }]
    )

    ld = LaunchDescription()

    ld.add_action(declare_robot_id)
    ld.add_action(declare_checkTerrainConn)
    ld.add_action(terrain_analysis_ext_node)
    
    return ld