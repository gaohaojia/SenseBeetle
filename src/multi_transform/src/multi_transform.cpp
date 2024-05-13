#include <functional>
#include <rclcpp/node_options.hpp>

#include "multi_transform/multi_transform.hpp"

namespace multi_transform
{
MultiTransformNode::MultiTransformNode(const rclcpp::NodeOptions& options)
  : Node("multi_transform", options)
{
  this->declare_parameter<int>("robot_id", 0);
  this->get_parameter("robot_id", robot_id);

  terrain_map_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("terrain_map_", 2, std::bind(&MultiTransformNode::TerrainMapCallBack, this, std::placeholders::_1));
  terrain_map_ext_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("terrain_map_ext_", 2, std::bind(&MultiTransformNode::TerrainMapExtCallBack, this, std::placeholders::_1));
  total_terrain_map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/total_terrain_map", 2);
  total_terrain_map_ext_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/total_terrain_map_ext", 2);
}

void MultiTransformNode::TerrainMapCallBack(const sensor_msgs::msg::PointCloud2::ConstSharedPtr terrain_map_msg){

}

void MultiTransformNode::TerrainMapExtCallBack(const sensor_msgs::msg::PointCloud2::ConstSharedPtr terrain_map_ext_msg){

}
} // namespace multi_transform