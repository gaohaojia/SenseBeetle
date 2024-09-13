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
#include <sensor_msgs/msg/imu.hpp>

namespace lidar_transform
{
class LidarTransform : public rclcpp::Node
{
public:
  LidarTransform(const rclcpp::NodeOptions & options);

private:
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  tf2::Transform transform;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg);
};
} // namespace lidar_transform

#endif