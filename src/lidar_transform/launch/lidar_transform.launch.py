from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import LaunchConfigurationEquals
import os
import yaml


def generate_launch_description():
    robot_type = LaunchConfiguration("robot_type")

    declare_robot_type = DeclareLaunchArgument(
        "robot_type", default_value="simulated", description=""
    )

    if LaunchConfigurationEquals(robot_type, "v4"):
        params_file = os.path.join(
            get_package_share_directory("lidar_transform"),
            "config",
            "lidar_transform_params_v4.yaml",
        )
    else:
        params_file = os.path.join(
            get_package_share_directory("lidar_transform"),
            "config",
            "lidar_transform_params_simulated.yaml",
        )

    with open(params_file, "r") as file:
        offset_params = yaml.safe_load(file)["lidar_transform"]["ros__parameters"]

    lidar_positionX = offset_params["lidar_positionX"]
    lidar_positionY = offset_params["lidar_positionY"]
    lidar_positionZ = offset_params["lidar_positionZ"]
    lidar_rotateR = offset_params["lidar_rotateR"]
    lidar_rotateP = offset_params["lidar_rotateP"]
    lidar_rotateY = offset_params["lidar_rotateY"]

    camera_positionX = offset_params["camera_positionX"]
    camera_positionY = offset_params["camera_positionY"]
    camera_positionZ = offset_params["camera_positionZ"]
    camera_rotateR = offset_params["camera_rotateR"]
    camera_rotateP = offset_params["camera_rotateP"]
    camera_rotateY = offset_params["camera_rotateY"]

    lidar_transform_node = Node(
        package="lidar_transform",
        executable="lidar_transform_node",
        name="lidar_transform",
        output="screen",
        parameters=[params_file],
    )

    lidarOffsetList_str = list(
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

    cameraOffsetList_str = list(
        map(
            str,
            [
                camera_positionX,
                camera_positionY,
                camera_positionZ,
                camera_rotateY,
                camera_rotateP,
                camera_rotateR,
            ],
        )
    )

    lidar_trans_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="lidarTransPublisher",
        arguments=[*lidarOffsetList_str, "base_link", "lidar"],
    )

    realsense_trans_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="mapTransPublisher",
        arguments=[*cameraOffsetList_str, "base_link", "camera_link"],
    )

    ld = LaunchDescription()
    ld.add_action(declare_robot_type)

    ld.add_action(lidar_trans_publisher)
    ld.add_action(realsense_trans_publisher)
    ld.add_action(TimerAction(period=10.0, actions=[lidar_transform_node]))

    return ld
