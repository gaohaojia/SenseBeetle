#ifndef IMU_TRANSFORM_HPP
#define IMU_TRANSFORM_HPP

#include <memory>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>

namespace lidar_transform
{
class LidarTransform : public rclcpp::Node
{
public:
  LidarTransform(const rclcpp::NodeOptions & options);

private:
  bool enable_pid = false;
  double pid_kp;
  double pid_ki;
  double pid_kd;

  geometry_msgs::msg::Twist pid_current_velocity;
  geometry_msgs::msg::Twist pid_target_velocity;
  geometry_msgs::msg::Twist pid_prev_error;
  geometry_msgs::msg::Twist pid_integral;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  tf2::Transform frame_transform;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr lidar_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr lidar_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg);
  void lidar_callback(const livox_ros_driver2::msg::CustomMsg::SharedPtr lidar_msg);
  void cmd_vel_callback(const geometry_msgs::msg::TwistStamped::SharedPtr cmd_vel_msg);
};
} // namespace lidar_transform

#endif