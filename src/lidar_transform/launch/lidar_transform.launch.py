from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def get_id_map_trans_publisher(context: LaunchContext, imuOffsetList):
    imuOffsetList_str = []
    for idx in imuOffsetList:
        imuOffsetList_str.append(context.perform_substitution(idx))
    map_trans_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="imuTransPublisher",
        arguments=[*imuOffsetList_str, "lidar", "imu_link"],
    )
    return [map_trans_publisher]


def generate_launch_description():
    lidarOffsetPositionX = LaunchConfiguration("lidarOffsetPositionX")
    lidarOffsetPositionY = LaunchConfiguration("lidarOffsetPositionY")
    lidarOffsetPositionZ = LaunchConfiguration("lidarOffsetPositionZ")
    lidarOffsetRotateR = LaunchConfiguration("lidarOffsetRotateR")
    lidarOffsetRotateP = LaunchConfiguration("lidarOffsetRotateP")
    lidarOffsetRotateY = LaunchConfiguration("lidarOffsetRotateY")

    declare_lidarOffsetPositionX = DeclareLaunchArgument(
        "lidarOffsetPositionX", default_value="0.0", description=""
    )
    declare_lidarOffsetPositionY = DeclareLaunchArgument(
        "lidarOffsetPositionY", default_value="0.0", description=""
    )
    declare_lidarOffsetPositionZ = DeclareLaunchArgument(
        "lidarOffsetPositionZ", default_value="0.0", description=""
    )
    declare_lidarOffsetRotateR = DeclareLaunchArgument(
        "lidarOffsetRotateR", default_value="0.0", description=""
    )
    declare_lidarOffsetRotateP = DeclareLaunchArgument(
        "lidarOffsetRotateP", default_value="0.0", description=""
    )
    declare_lidarOffsetRotateY = DeclareLaunchArgument(
        "lidarOffsetRotateY", default_value="0.0", description=""
    )

    lidar_transform_node = Node(
        package="lidar_transform",
        executable="lidar_transform_node",
        name="lidar_transform",
        output="screen",
        parameters=[
            {
                "lidarOffsetPositionX": lidarOffsetPositionX,
                "lidarOffsetPositionY": lidarOffsetPositionY,
                "lidarOffsetPositionZ": lidarOffsetPositionZ,
                "lidarOffsetRotateR": lidarOffsetRotateR,
                "lidarOffsetRotateP": lidarOffsetRotateP,
                "lidarOffsetRotateY": lidarOffsetRotateY,
            }
        ],
    )

    ld = LaunchDescription()

    ld.add_action(declare_lidarOffsetPositionX)
    ld.add_action(declare_lidarOffsetPositionY)
    ld.add_action(declare_lidarOffsetPositionZ)
    ld.add_action(declare_lidarOffsetRotateR)
    ld.add_action(declare_lidarOffsetRotateP)
    ld.add_action(declare_lidarOffsetRotateY)

    ld.add_action(
        OpaqueFunction(
            function=get_id_map_trans_publisher,
            args=[
                [
                    lidarOffsetPositionX,
                    lidarOffsetPositionY,
                    lidarOffsetPositionZ,
                    lidarOffsetRotateY,
                    lidarOffsetRotateP,
                    lidarOffsetRotateR,
                ]
            ],
        )
    )
    ld.add_action(TimerAction(period=10.0, actions=[lidar_transform_node]))

    return ld
