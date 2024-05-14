#include <functional>
#include <geometry_msgs/msg/detail/point_stamped__struct.hpp>
#include <memory>
#include <nav_msgs/msg/detail/odometry__struct.hpp>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/detail/point_cloud2__struct.hpp>
#include <string>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/convert.h>
#include <tf2/time.h>
#include <tf2/transform_datatypes.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "multi_transform/multi_transform.hpp"

namespace multi_transform
{
MultiTransformNode::MultiTransformNode(const rclcpp::NodeOptions& options)
  : Node("multi_transform", options)
{
  this->declare_parameter<int>("robot_id", 0);
  this->get_parameter("robot_id", robot_id);

  terrain_map_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "terrain_map",
    2,
    std::bind(&MultiTransformNode::TerrainMapCallBack, this, std::placeholders::_1));
  terrain_map_ext_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "terrain_map_ext",
    2,
    std::bind(&MultiTransformNode::TerrainMapExtCallBack, this, std::placeholders::_1));
  registered_scan_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "registered_scan",
    5,
    std::bind(&MultiTransformNode::RegisteredScanCallBack, this, std::placeholders::_1));
  state_estimation_at_scan_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "state_estimation_at_scan",
    5,
    std::bind(&MultiTransformNode::StateEstimationAtScanCallBack, this, std::placeholders::_1));
  way_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "way_point",
    2,
    std::bind(&MultiTransformNode::WayPointCallBack, this, std::placeholders::_1));

  total_terrain_map_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("/total_terrain_map", 2);
  total_terrain_map_ext_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("/total_terrain_map_ext", 2);
  total_registered_scan_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("total_registered_scan", 5);
  total_state_estimation_at_scan_pub_ =
    this->create_publisher<nav_msgs::msg::Odometry>("total_state_estimation_at_scan", 5);
  local_way_point_pub_ =
    this->create_publisher<geometry_msgs::msg::PointStamped>("local_way_point", 2);

  std::string fromFrameRel = "robot_" + std::to_string(robot_id) + "/map";
  std::string toFrameRel = "map";
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  try {
    transformStamped = std::make_shared<geometry_msgs::msg::TransformStamped>(
      tf_buffer_->lookupTransform(toFrameRel, fromFrameRel, tf2::TimePointZero, tf2::durationFromSec(10.0)));
  } catch (const tf2::TransformException& ex) {
    RCLCPP_INFO(this->get_logger(),
                "Could not transform %s to %s: %s",
                toFrameRel.c_str(),
                fromFrameRel.c_str(),
                ex.what());
    return;
  }
  fromIdMapToMap = std::make_shared<Eigen::Matrix4d>(
    tf2::transformToEigen(transformStamped->transform).matrix().cast<double>());
}

void MultiTransformNode::TerrainMapCallBack(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr terrain_map_msg)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_tmp(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_result(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::fromROSMsg(*terrain_map_msg, *pointcloud_tmp);
  pcl::transformPointCloud(*pointcloud_tmp, *pointcloud_result, *fromIdMapToMap);
  std::shared_ptr<sensor_msgs::msg::PointCloud2> totalTerrainCloud;
  pcl::toROSMsg(*pointcloud_result, *totalTerrainCloud);
  totalTerrainCloud->header.stamp = terrain_map_msg->header.stamp;
  totalTerrainCloud->header.frame_id = "map";
  total_terrain_map_pub_->publish(*totalTerrainCloud);
}

void MultiTransformNode::TerrainMapExtCallBack(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr terrain_map_ext_msg)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_tmp(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_result(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::fromROSMsg(*terrain_map_ext_msg, *pointcloud_tmp);
  pcl::transformPointCloud(*pointcloud_tmp, *pointcloud_result, *fromIdMapToMap);
  std::shared_ptr<sensor_msgs::msg::PointCloud2> totalTerrainExtCloud;
  pcl::toROSMsg(*pointcloud_result, *totalTerrainExtCloud);
  totalTerrainExtCloud->header.stamp = terrain_map_ext_msg->header.stamp;
  totalTerrainExtCloud->header.frame_id = "map";
  total_terrain_map_ext_pub_->publish(*totalTerrainExtCloud);
}

void MultiTransformNode::RegisteredScanCallBack(const sensor_msgs::msg::PointCloud2::ConstSharedPtr registered_scan_msg){
  pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_tmp(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_result(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::fromROSMsg(*registered_scan_msg, *pointcloud_tmp);
  pcl::transformPointCloud(*pointcloud_tmp, *pointcloud_result, *fromIdMapToMap);
  std::shared_ptr<sensor_msgs::msg::PointCloud2> registeredScanCloud;
  pcl::toROSMsg(*pointcloud_result, *registeredScanCloud);
  registeredScanCloud->header.stamp = registered_scan_msg->header.stamp;
  registeredScanCloud->header.frame_id = "map";
  total_registered_scan_pub_->publish(*registeredScanCloud);
}

void MultiTransformNode::StateEstimationAtScanCallBack(const nav_msgs::msg::Odometry::ConstSharedPtr state_estimation_at_scan_msg){
  total_state_estimation_at_scan_pub_->publish(*state_estimation_at_scan_msg);
}

void MultiTransformNode::WayPointCallBack(const geometry_msgs::msg::PointStamped::ConstSharedPtr way_point_msg){
  std::shared_ptr<geometry_msgs::msg::PointStamped> local_point;
  local_point = std::make_shared<geometry_msgs::msg::PointStamped>(tf_buffer_->transform(*way_point_msg, "robot_" + std::to_string(robot_id) + "/map"));
  local_way_point_pub_->publish(*local_point);
}
} // namespace multi_transform

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(multi_transform::MultiTransformNode)