#ifndef LIDAR_TRANSFORM_HPP
#define LIDAR_TRANSFORM_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <memory>
#include <pcl_ros/transforms.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace lidar_transform {
class LidarTransform : public rclcpp::Node {
 public:
  LidarTransform(const rclcpp::NodeOptions& options);

 private:
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  tf2::Transform frame_transform;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr
    livox_lidar_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_pub_;
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr
    livox_lidar_pub_;

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg);
  void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr lidar_msg);
  void livox_lidar_callback(
    const livox_ros_driver2::msg::CustomMsg::SharedPtr lidar_msg);
};
}  // namespace lidar_transform

#endif