import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import LaunchConfigurationEquals


def generate_launch_description():
    robot_id = LaunchConfiguration("robot_id")
    checkTerrainConn = LaunchConfiguration("checkTerrainConn")
    lidar_type = LaunchConfiguration("lidar_type")

    declare_robot_id = DeclareLaunchArgument(
        "robot_id", default_value="0", description=""
    )
    declare_planner_mode = DeclareLaunchArgument(
        "planner_mode", default_value="none", description=""
    )
    declare_checkTerrainConn = DeclareLaunchArgument(
        "checkTerrainConn", default_value="true", description=""
    )
    declare_lidar_type = DeclareLaunchArgument(
        "lidar_type", default_value="mid360", description=""
    )

    optional_nodes = []
    try:
        start_livox_mid360 = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("livox_ros_driver2"),
                    "launch",
                    "msg_MID360_launch.py",
                )
            )
        )
        optional_nodes.append(start_livox_mid360)
    except:
        print("Not found livox_ros_driver2 package.")

    try:
        start_realsense = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("realsense2_camera"),
                    "launch",
                    "rs_launch.py",
                )
            ),
            launch_arguments={"pointcloud.enable": "true"}.items(),
        )
        optional_nodes.append(start_realsense)
    except:
        print("Not found realsense2_camera package.")

    try:
        start_std = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("std_detector"),
                    "launch",
                    "std_detector.launch.py",
                )
            )
        )
        optional_nodes.append(start_std)
    except:
        print("Not found std_detector package.")

    try:
        start_tare_planner = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("tare_planner"),
                    "launch",
                    "multi_explore.launch.py",
                )
            ),
            condition=LaunchConfigurationEquals("planner_mode", "tare_planner"),
        )
        optional_nodes.append(start_tare_planner)
    except:
        print("Not found tare_planner package.")

    start_lidar_transform = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("lidar_transform"),
                "launch",
                "lidar_transform.launch.py",
            )
        )
    )

    start_point_lio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("point_lio"),
                "launch",
                "point_lio.launch.py",
            )
        ),
        launch_arguments={
            "lidar_type": lidar_type
        }.items(),
    )

    start_robot_communication = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("robot_communication"),
                "launch",
                "robot_communication.launch.py",
            )
        ),
        launch_arguments={
            "robot_id": robot_id,
        }.items(),
    )

    start_local_planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("local_planner"),
                "launch",
                "local_planner.launch.py",
            )
        )
    )

    start_terrain_analysis = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("terrain_analysis"),
                "launch",
                "terrain_analysis.launch.py",
            )
        ),
    )

    start_terrain_analysis_ext = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("terrain_analysis_ext"),
                "launch",
                "terrain_analysis_ext.launch.py",
            )
        ),
        launch_arguments={
            "checkTerrainConn": checkTerrainConn,
        }.items(),
    )

    start_sensor_scan_generation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("sensor_scan_generation"),
                "launch",
                "sensor_scan_generation.launch.py",
            )
        ),
    )

    start_loam_interface = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("loam_interface"),
                "launch",
                "loam_interface.launch.py",
            )
        ),
    )

    start_rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            os.path.join(
                get_package_share_directory("local_bringup"), "rviz", "real_robot.rviz"
            ),
        ],
        output="screen",
    )

    ld = LaunchDescription()

    # Add the actions
    ld.add_action(declare_robot_id)
    ld.add_action(declare_lidar_type)
    ld.add_action(declare_planner_mode)
    ld.add_action(declare_checkTerrainConn)

    for node in optional_nodes:
        try:
            ld.add_action(node)
        except:
            continue
    ld.add_action(start_lidar_transform)
    ld.add_action(start_point_lio)
    ld.add_action(TimerAction(period=10.0, actions=[start_robot_communication]))
    ld.add_action(start_local_planner)
    ld.add_action(start_terrain_analysis)
    ld.add_action(start_terrain_analysis_ext)
    ld.add_action(start_sensor_scan_generation)
    ld.add_action(start_loam_interface)
    ld.add_action(TimerAction(period=8.0, actions=[start_rviz]))

    return ld
