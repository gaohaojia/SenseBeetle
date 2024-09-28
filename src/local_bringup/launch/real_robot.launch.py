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

    declare_robot_id = DeclareLaunchArgument(
        "robot_id", default_value="0", description=""
    )
    declare_lio_mode = DeclareLaunchArgument(
        "lio_mode", default_value="point_lio", description=""
    )
    declare_planner_mode = DeclareLaunchArgument(
        "planner_mode", default_value="none", description=""
    )
    declare_realsense_mode = DeclareLaunchArgument(
        "realsense_mode", default_value="none", description=""
    )
    declare_checkTerrainConn = DeclareLaunchArgument(
        "checkTerrainConn", default_value="true", description=""
    )

    start_livox_mid360 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("livox_ros_driver2"),
                "launch",
                "msg_MID360_launch.py",
            )
        )
    )

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
            condition=LaunchConfigurationEquals("realsense_mode", "rs_d435"),
        )
    except:
        pass

    start_lidar_transform = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("lidar_transform"),
                "launch",
                "lidar_transform.launch.py",
            )
        )
    )

    start_fast_lio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("fast_lio"),
                "launch",
                "mapping_mid360.launch.py",
            )
        ),
        condition=LaunchConfigurationEquals("lio_mode", "fast_lio"),
    )

    start_point_lio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("point_lio"),
                "launch",
                "point_lio.launch.py",
            )
        ),
        condition=LaunchConfigurationEquals("lio_mode", "point_lio"),
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
    ld.add_action(declare_lio_mode)
    ld.add_action(declare_planner_mode)
    ld.add_action(declare_realsense_mode)
    ld.add_action(declare_checkTerrainConn)

    try:
        ld.add_action(start_realsense)
    except:
        pass
    ld.add_action(start_livox_mid360)
    ld.add_action(start_lidar_transform)
    ld.add_action(start_fast_lio)
    ld.add_action(start_point_lio)
    ld.add_action(TimerAction(period=10.0, actions=[start_robot_communication]))
    ld.add_action(start_local_planner)
    ld.add_action(start_terrain_analysis)
    ld.add_action(start_terrain_analysis_ext)
    ld.add_action(start_sensor_scan_generation)
    ld.add_action(start_loam_interface)
    ld.add_action(start_tare_planner)
    ld.add_action(TimerAction(period=8.0, actions=[start_rviz]))

    return ld
