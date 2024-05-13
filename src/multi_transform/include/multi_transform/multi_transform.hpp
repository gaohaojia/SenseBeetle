#ifndef MULTI_TRANSFORM
#define MULTI_TRANSFORM

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>

#include "sensor_msgs/msg/point_cloud2.hpp"

namespace multi_transform
{
class MultiTransformNode : public rclcpp::Node
{
public:
  MultiTransformNode(const rclcpp::NodeOptions& options);

private:
  int robot_id;

  void TerrainMapCallBack(const sensor_msgs::msg::PointCloud2::ConstSharedPtr terrain_map_msg);
  void TerrainMapExtCallBack(const sensor_msgs::msg::PointCloud2::ConstSharedPtr terrain_map_ext_msg);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr terrain_map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr terrain_map_ext_sub_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr total_terrain_map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr total_terrain_map_ext_pub_;
};
}  // namespace multi_transform

#endif