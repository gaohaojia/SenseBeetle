import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import LaunchConfiguration

def get_vehicle_trans_publisher(context: LaunchContext, sensorOffsetX, sensorOffsetY, robot_id):
    robot_id_str = 'robot_' + context.perform_substitution(robot_id)
    sensorOffsetX_str = context.perform_substitution(sensorOffsetX)
    sensorOffsetY_str = context.perform_substitution(sensorOffsetY)
    sensorOffsetX_str = str(-float(sensorOffsetX_str))
    sensorOffsetY_str = str(-float(sensorOffsetY_str))
    vehicle_trans_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="vehicleTransPublisher",
        arguments=[sensorOffsetX_str, sensorOffsetY_str, '0', '0', '0', '0', robot_id_str + '/sensor', robot_id_str + '/vehicle']
    )
    return [vehicle_trans_publisher]

def get_sensor_trans_publisher(context: LaunchContext, cameraOffsetZ, robot_id):
    robot_id_str = 'robot_' + context.perform_substitution(robot_id)
    cameraOffsetZ_str = context.perform_substitution(cameraOffsetZ)
    sensor_trans_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="sensorTransPublisher",
        arguments=['0', '0', cameraOffsetZ_str, '-1.5707963', '0', '-1.5707963', robot_id_str + '/sensor', robot_id_str + '/camera']
    )
    return [sensor_trans_publisher]

def generate_launch_description():
    robot_id = LaunchConfiguration('robot_id')
    sensorOffsetX = LaunchConfiguration('sensorOffsetX')
    sensorOffsetY = LaunchConfiguration('sensorOffsetY')
    cameraOffsetZ = LaunchConfiguration('cameraOffsetZ')
    twoWayDrive = LaunchConfiguration('twoWayDrive')
    maxSpeed = LaunchConfiguration('maxSpeed')
    autonomyMode = LaunchConfiguration('autonomyMode')
    autonomySpeed = LaunchConfiguration('autonomySpeed')
    joyToSpeedDelay = LaunchConfiguration('joyToSpeedDelay')
    goalX = LaunchConfiguration('goalX')
    goalY = LaunchConfiguration('goalY')
    
    declare_robot_id = DeclareLaunchArgument('robot_id', default_value='0', description='')
    declare_sensorOffsetX = DeclareLaunchArgument('sensorOffsetX', default_value='0.0', description='')
    declare_sensorOffsetY = DeclareLaunchArgument('sensorOffsetY', default_value='0.0', description='')
    declare_cameraOffsetZ = DeclareLaunchArgument('cameraOffsetZ', default_value='0.0', description='')
    declare_twoWayDrive = DeclareLaunchArgument('twoWayDrive', default_value='true', description='')
    declare_maxSpeed = DeclareLaunchArgument('maxSpeed', default_value='2.0', description='')
    declare_autonomyMode = DeclareLaunchArgument('autonomyMode', default_value='true', description='')
    delcare_autonomySpeed = DeclareLaunchArgument('autonomySpeed', default_value='2.0', description='')
    declare_joyToSpeedDelay = DeclareLaunchArgument('joyToSpeedDelay', default_value='2.0', description='')
    declare_goalX = DeclareLaunchArgument('goalX', default_value='0.0', description='')
    declare_goalY = DeclareLaunchArgument('goalY', default_value='0.0', description='')
    
    local_planner_node = Node(
        package="local_planner",
        executable="localPlanner",
        name="localPlanner",
        output="screen",
        parameters=[{
            'robot_id': robot_id,
            "pathFolder" : os.path.join(get_package_share_directory('local_planner'), 'paths'),
            "vehicleLength" : 0.6,
            "vehicleWidth" : 0.6,
            "sensorOffsetX" : LaunchConfiguration('sensorOffsetX'),
            "sensorOffsetY" : LaunchConfiguration('sensorOffsetY'),
            "twoWayDrive" : LaunchConfiguration('twoWayDrive'),
            "laserVoxelSize" : 0.05,
            "terrainVoxelSize" : 0.2,
            "useTerrainAnalysis" : True,
            "checkObstacle" : True,
            "checkRotObstacle" : False,
            "adjacentRange" : 4.25,
            "obstacleHeightThre" : 0.15,
            "groundHeightThre" : 0.1,
            "costHeightThre" : 0.1,
            "costScore" : 0.02,
            "useCost" : False,
            "pointPerPathThre" : 2,
            "minRelZ" : -0.5,
            "maxRelZ" : 0.25,
            "maxSpeed" : LaunchConfiguration('maxSpeed'),
            "dirWeight" : 0.02,
            "dirThre" : 90.0,
            "dirToVehicle" : False,
            "pathScale" : 1.25,
            "minPathScale" : 0.75,
            "pathScaleStep" : 0.25,
            "pathScaleBySpeed" : True,
            "minPathRange" : 1.0,
            "pathRangeStep" : 0.5,
            "pathRangeBySpeed" : True,
            "pathCropByGoal" : True,
            "autonomyMode" : LaunchConfiguration('autonomyMode'),
            "autonomySpeed" : LaunchConfiguration('autonomySpeed'),
            "joyToSpeedDelay" : LaunchConfiguration('autonomySpeed'),
            "joyToCheckObstacleDelay" : 5.0,
            "goalClearRange" : 0.5,
            "goalX" : LaunchConfiguration('goalX'),
            "goalY" : LaunchConfiguration('goalY')
        }]
    )

    path_follower_node = Node(
        package="local_planner",
        executable="pathFollower",
        name="pathFollower",
        output="screen",
        parameters=[{
            'robot_id': robot_id,
            "sensorOffsetX" : LaunchConfiguration('sensorOffsetX'),
            "sensorOffsetY" : LaunchConfiguration('sensorOffsetY'),
            "pubSkipNum" : 1,
            "twoWayDrive" : LaunchConfiguration('twoWayDrive'),
            "lookAheadDis" : 0.5,
            "yawRateGain" : 7.5,
            "stopYawRateGain" : 7.5,
            "maxYawRate" : 90.0,
            "maxSpeed" : LaunchConfiguration('maxSpeed'),
            "maxAccel" : 2.5,
            "switchTimeThre" : 1.0,
            "dirDiffThre" : 0.1,
            "stopDisThre" : 0.2,
            "slowDwnDisThre" : 0.85,
            "useInclRateToSlow" : False,
            "inclRateThre" : 120.0,
            "slowRate1" : 0.25,
            "slowRate2" : 0.5,
            "slowTime1" : 2.0,
            "slowTime2" : 2.0,
            "useInclToStop" : False,
            "inclThre" : 45.0,
            "stopTime" : 5.0,
            "noRotAtStop" : False,
            "noRotAtGoal" : True,
            "autonomyMode" : LaunchConfiguration('autonomyMode'),
            "autonomySpeed" : LaunchConfiguration('autonomySpeed'),
            "joyToSpeedDelay" : LaunchConfiguration('autonomySpeed')
        }]
    )

    ld = LaunchDescription()

    # Add the actions
    ld.add_action(declare_robot_id)
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

    ld.add_action(OpaqueFunction(function=get_vehicle_trans_publisher, args=[sensorOffsetX, sensorOffsetY, robot_id]))
    ld.add_action(OpaqueFunction(function=get_sensor_trans_publisher, args=[cameraOffsetZ, robot_id]))
    return ld