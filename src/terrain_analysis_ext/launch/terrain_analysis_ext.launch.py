from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    checkTerrainConn = LaunchConfiguration("checkTerrainConn")
    declare_checkTerrainConn = DeclareLaunchArgument(
        "checkTerrainConn", default_value="true", description=""
    )

    terrain_analysis_ext_node = Node(
        package="terrain_analysis_ext",
        executable="terrainAnalysisExt",
        name="terrainAnalysisExt",
        output="screen",
        parameters=[
            {
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
                "localTerrainMapRadius": 4.0,
            }
        ],
    )

    ld = LaunchDescription()

    ld.add_action(declare_checkTerrainConn)
    ld.add_action(terrain_analysis_ext_node)

    return ld
