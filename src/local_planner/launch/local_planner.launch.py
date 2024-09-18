import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def get_vehicle_trans_publisher(context: LaunchContext, sensorOffsetX, sensorOffsetY):
    sensorOffsetX_str = context.perform_substitution(sensorOffsetX)
    sensorOffsetY_str = context.perform_substitution(sensorOffsetY)
    sensorOffsetX_str = str(-float(sensorOffsetX_str))
    sensorOffsetY_str = str(-float(sensorOffsetY_str))
    vehicle_trans_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="vehicleTransPublisher",
        arguments=[
            sensorOffsetX_str,
            sensorOffsetY_str,
            "0",
            "0",
            "0",
            "0",
            "sensor",
            "vehicle",
        ],
    )
    return [vehicle_trans_publisher]


def get_sensor_trans_publisher(context: LaunchContext, cameraOffsetZ):
    cameraOffsetZ_str = context.perform_substitution(cameraOffsetZ)
    sensor_trans_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="sensorTransPublisher",
        arguments=[
            "0",
            "0",
            cameraOffsetZ_str,
            "-1.5707963",
            "0",
            "-1.5707963",
            "sensor",
            "camera",
        ],
    )
    return [sensor_trans_publisher]


def generate_launch_description():
    common_params_file = os.path.join(
        get_package_share_directory("local_planner"), "config", "common_params.yaml"
        # get_package_share_directory("local_planner"), "config", "common_params_v3.yaml"
    )
    with open(common_params_file, "r") as file:
        params = yaml.safe_load(file)

    sensorOffsetX = LaunchConfiguration("sensorOffsetX")
    sensorOffsetY = LaunchConfiguration("sensorOffsetY")
    cameraOffsetZ = LaunchConfiguration("cameraOffsetZ")
    twoWayDrive = LaunchConfiguration("twoWayDrive")
    maxSpeed = LaunchConfiguration("maxSpeed")
    autonomyMode = LaunchConfiguration("autonomyMode")
    autonomySpeed = LaunchConfiguration("autonomySpeed")
    joyToSpeedDelay = LaunchConfiguration("joyToSpeedDelay")
    goalX = LaunchConfiguration("goalX")
    goalY = LaunchConfiguration("goalY")

    declare_sensorOffsetX = DeclareLaunchArgument(
        "sensorOffsetX", default_value=str(params["sensorOffsetX"]), description=""
    )
    declare_sensorOffsetY = DeclareLaunchArgument(
        "sensorOffsetY", default_value=str(params["sensorOffsetY"]), description=""
    )
    declare_cameraOffsetZ = DeclareLaunchArgument(
        "cameraOffsetZ", default_value=str(params["cameraOffsetZ"]), description=""
    )
    declare_twoWayDrive = DeclareLaunchArgument(
        "twoWayDrive", default_value=str(params["twoWayDrive"]), description=""
    )
    declare_maxSpeed = DeclareLaunchArgument(
        "maxSpeed", default_value=str(params["maxSpeed"]), description=""
    )
    declare_autonomyMode = DeclareLaunchArgument(
        "autonomyMode", default_value=str(params["autonomyMode"]), description=""
    )
    delcare_autonomySpeed = DeclareLaunchArgument(
        "autonomySpeed", default_value=str(params["autonomySpeed"]), description=""
    )
    declare_joyToSpeedDelay = DeclareLaunchArgument(
        "joyToSpeedDelay", default_value=str(params["joyToSpeedDelay"]), description=""
    )
    declare_goalX = DeclareLaunchArgument("goalX", default_value="0.0", description="")
    declare_goalY = DeclareLaunchArgument("goalY", default_value="0.0", description="")

    local_planner_node = Node(
        package="local_planner",
        executable="localPlanner",
        name="localPlanner",
        output="screen",
        parameters=[
            {
                "pathFolder": os.path.join(
                    get_package_share_directory("local_planner"), "paths", "original"
                    # get_package_share_directory("local_planner"), "paths", "v3"
                ),
                "sensorOffsetX": sensorOffsetX,
                "sensorOffsetY": sensorOffsetY,
                "twoWayDrive": twoWayDrive,
                "maxSpeed": maxSpeed,
                "autonomyMode": autonomyMode,
                "autonomySpeed": autonomySpeed,
                "joyToSpeedDelay": joyToSpeedDelay,
                "goalX": goalX,
                "goalY": goalY,
            },
            os.path.join(
                get_package_share_directory("local_planner"),
                "config",
                "local_planner.yaml",
                # "local_planner_v3.yaml",
            ),
        ],
    )

    path_follower_node = Node(
        package="local_planner",
        executable="pathFollower",
        name="pathFollower",
        output="screen",
        parameters=[
            {
                "sensorOffsetX": sensorOffsetX,
                "sensorOffsetY": sensorOffsetY,
                "twoWayDrive": twoWayDrive,
                "maxSpeed": maxSpeed,
                "autonomyMode": autonomyMode,
                "autonomySpeed": autonomySpeed,
                "joyToSpeedDelay": joyToSpeedDelay,
            },
            os.path.join(
                get_package_share_directory("local_planner"),
                "config",
                "path_follower.yaml",
                # "path_follower_v3.yaml",
            ),
        ],
    )

    ld = LaunchDescription()

    # Add the actions
    ld.add_action(declare_sensorOffsetX)
    ld.add_action(declare_sensorOffsetY)
    ld.add_action(declare_cameraOffsetZ)
    ld.add_action(declare_twoWayDrive)
    ld.add_action(declare_maxSpeed)
    ld.add_action(declare_autonomyMode)
    ld.add_action(delcare_autonomySpeed)
    ld.add_action(declare_joyToSpeedDelay)
    ld.add_action(declare_goalX)
    ld.add_action(declare_goalY)

    ld.add_action(local_planner_node)
    ld.add_action(path_follower_node)

    ld.add_action(
        OpaqueFunction(
            function=get_vehicle_trans_publisher, args=[sensorOffsetX, sensorOffsetY]
        )
    )
    ld.add_action(
        OpaqueFunction(function=get_sensor_trans_publisher, args=[cameraOffsetZ])
    )
    return ld
