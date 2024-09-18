from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os
import yaml


def generate_launch_description():
    robot_id = LaunchConfiguration("robot_id")

    declare_robot_id = DeclareLaunchArgument(
        "robot_id", default_value="0", description=""
    )

    params_file = os.path.join(
        get_package_share_directory("robot_communication"), "config", "robot_communication_params.yaml"
    )
    with open(params_file, "r") as file:
        offset_params = yaml.safe_load(file)["robot_communication"]["ros__parameters"]

    multi_positionX = offset_params["multi_positionX"]
    multi_positionY = offset_params["multi_positionY"]
    multi_positionZ = offset_params["multi_positionZ"]
    multi_rotateR = offset_params["multi_rotateR"]
    multi_rotateP = offset_params["multi_rotateP"]
    multi_rotateY = offset_params["multi_rotateY"]

    robot_communication_node = Node(
        package="robot_communication",
        executable="robot_communication_node",
        name="robot_communication",
        output="screen",
        respawn=True,
        parameters=[
            {
                "robot_id": robot_id,
            },
            os.path.join(
                get_package_share_directory("robot_communication"),
                "config",
                "robot_communication_params.yaml",
            ),
        ],
    )

    offsetList_str = list(
        map(
            str,
            [
                multi_positionX,
                multi_positionY,
                multi_positionZ,
                multi_rotateY,
                multi_rotateP,
                multi_rotateR,
            ],
        )
    )
    map_trans_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="idMapTransPublisher",
        arguments=[*offsetList_str, "map", "local_map"],
    )

    ld = LaunchDescription()

    ld.add_action(declare_robot_id)

    ld.add_action(robot_communication_node)
    ld.add_action(map_trans_publisher)

    return ld
