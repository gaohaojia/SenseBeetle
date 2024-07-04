source install/setup.sh
export ROS_DOMAIN_ID=$(expr $1 + 1)
ros2 launch bringup real_robot.launch.py robot_id:=$1