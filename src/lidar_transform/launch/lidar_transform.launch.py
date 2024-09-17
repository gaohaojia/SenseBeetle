from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os
import yaml


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory("lidar_transform"),
        "config",
        "lidar_transform_params.yaml",
    )
    with open(params_file, "r") as file:
        offset_params = yaml.safe_load(file)["lidar_transform"]["ros__parameters"]

    lidar_positionX = offset_params["lidar_positionX"]
    lidar_positionY = offset_params["lidar_positionY"]
    lidar_positionZ = offset_params["lidar_positionZ"]
    lidar_rotateR = offset_params["lidar_rotateR"]
    lidar_rotateP = offset_params["lidar_rotateP"]
    lidar_rotateY = offset_params["lidar_rotateY"]

    lidar_transform_node = Node(
        package="lidar_transform",
        executable="lidar_transform_node",
        name="lidar_transform",
        output="screen",
        parameters=[
            os.path.join(
                get_package_share_directory("lidar_transform"),
                "config",
                "lidar_transform_params.yaml",
            ),
        ],
    )

    offsetList_str = list(
        map(
            str,
            [
                lidar_positionX,
                lidar_positionY,
                lidar_positionZ,
                lidar_rotateY,
                lidar_rotateP,
                lidar_rotateR,
            ],
        )
    )
    map_trans_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="idMapTransPublisher",
        arguments=[*offsetList_str, "lidar", "imu_link"],
    )

    ld = LaunchDescription()

    ld.add_action(map_trans_publisher)
    ld.add_action(TimerAction(period=10.0, actions=[lidar_transform_node]))

    return ld
