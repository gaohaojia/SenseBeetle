#include <chrono>
#include <cstring>
#include <functional>
#include <geometry_msgs/msg/detail/point_stamped__struct.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <netinet/in.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <rmw/qos_profiles.h>
#include <rmw/rmw.h>
#include <rmw/types.h>
#include <string>
#include <sys/socket.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/convert.h>
#include <tf2/time.h>
#include <tf2/transform_datatypes.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>

#include "multi_transform/multi_transform.hpp"

namespace multi_transform
{
MultiTransformNode::MultiTransformNode(const rclcpp::NodeOptions & options)
  : Node("multi_transform", options)
{
  this->declare_parameter<int>("robot_id", 0);
  this->declare_parameter<double>("multiOffsetPositionX", 0.0);
  this->declare_parameter<double>("multiOffsetPositionY", 0.0);
  this->declare_parameter<double>("multiOffsetPositionZ", 0.0);
  this->declare_parameter<double>("multiOffsetRotateR", 0.0);
  this->declare_parameter<double>("multiOffsetRotateP", 0.0);
  this->declare_parameter<double>("multiOffsetRotateY", 0.0);

  this->get_parameter("robot_id", robot_id);
  this->get_parameter("multiOffsetPositionX", multiOffsetPositionX);
  this->get_parameter("multiOffsetPositionY", multiOffsetPositionY);
  this->get_parameter("multiOffsetPositionZ", multiOffsetPositionZ);
  this->get_parameter("multiOffsetRotateR", multiOffsetRotateR);
  this->get_parameter("multiOffsetRotateP", multiOffsetRotateP);
  this->get_parameter("multiOffsetRotateY", multiOffsetRotateY);

  registered_scan_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "registered_scan",
    5,
    std::bind(&MultiTransformNode::RegisteredScanCallBack, this, std::placeholders::_1));
  way_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "way_point", 2, std::bind(&MultiTransformNode::WayPointCallBack, this, std::placeholders::_1));
  // terrain_map_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
  //   "terrain_map",
  //   2,
  //   std::bind(&MultiTransformNode::TerrainMapCallBack, this, std::placeholders::_1));
  // terrain_map_ext_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
  //   "terrain_map_ext",
  //   2,
  //   std::bind(&MultiTransformNode::TerrainMapExtCallBack, this, std::placeholders::_1));
  // state_estimation_at_scan_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
  //   "state_estimation_at_scan",
  //   5,
  //   std::bind(&MultiTransformNode::StateEstimationAtScanCallBack, this, std::placeholders::_1));

  total_registered_scan_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "total_registered_scan", rclcpp::SensorDataQoS());
  local_way_point_pub_ =
    this->create_publisher<geometry_msgs::msg::PointStamped>("local_way_point", 2);
  // total_terrain_map_pub_ =
  //   this->create_publisher<sensor_msgs::msg::PointCloud2>("total_terrain_map", 2);
  // total_terrain_map_ext_pub_ =
  //   this->create_publisher<sensor_msgs::msg::PointCloud2>("total_terrain_map_ext", 2);
  // total_state_estimation_at_scan_pub_ =
  //   this->create_publisher<nav_msgs::msg::Odometry>("total_state_estimation_at_scan", 5);

  std::string fromFrameRel = "robot_" + std::to_string(robot_id) + "/map";
  std::string toFrameRel = "map";
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  try {
    transformStamped =
      std::make_shared<geometry_msgs::msg::TransformStamped>(tf_buffer_->lookupTransform(
        toFrameRel, fromFrameRel, tf2::TimePointZero, tf2::durationFromSec(10.0)));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO(this->get_logger(),
                "Could not transform %s to %s: %s",
                toFrameRel.c_str(),
                fromFrameRel.c_str(),
                ex.what());
    return;
  }
  fromIdMapToMap = std::make_shared<Eigen::Matrix4d>(
    tf2::transformToEigen(transformStamped->transform).matrix().cast<double>());

  if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0){
    RCLCPP_ERROR(this->get_logger(), "Socket creation failed!");
    return;
  }

  memset(&server_addr, 0, sizeof(server_addr));

  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(port);
  server_addr.sin_addr.s_addr = INADDR_ANY;

  send_thread_ = std::thread(&MultiTransformNode::SendTotalRegisteredScan, this);
  recv_thread_ = std::thread(&MultiTransformNode::NetworkThread, this);
  RCLCPP_INFO(this->get_logger(), "Finish init multi transform node.");
}

MultiTransformNode::~MultiTransformNode()
{
  if (send_thread_.joinable()) {
    send_thread_.join();
  }
  if (recv_thread_.joinable()){
    recv_thread_.join();
  }
}

void MultiTransformNode::SendTotalRegisteredScan()
{
  int len = sizeof(server_addr);
  while (rclcpp::ok()) {
    rclcpp::sleep_for(std::chrono::nanoseconds(100));
    if (!total_registered_scan_queue.empty()) {
      std::shared_ptr<sensor_msgs::msg::PointCloud2> totalRegisteredScan =
        std::make_shared<sensor_msgs::msg::PointCloud2>(total_registered_scan_queue.front());
      total_registered_scan_queue.pop();
      // total_registered_scan_pub_->publish(*totalRegisteredScan);

      sendto(sockfd, buffer, strlen(buffer), MSG_CONFIRM, (const struct sockaddr *)&server_addr, len);
    }
    sendto(sockfd, "hello world", strlen("hello world"), MSG_WAITALL, (const struct sockaddr *)&server_addr, len);
  }
}

void MultiTransformNode::NetworkThread(){
  int n, len = sizeof(server_addr);
  while (rclcpp::ok()){
    n = recvfrom(sockfd, buffer, BUFFER_SIZE, 0, (struct sockaddr *)&server_addr, (socklen_t *)&len);
    buffer[n] = '\0';

    RCLCPP_INFO(this->get_logger(), "%s", buffer);
  }
}

void MultiTransformNode::RegisteredScanCallBack(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr registered_scan_msg)
{
  if (total_registered_scan_queue.size() >= 5) {
    total_registered_scan_queue.pop();
  }
  total_registered_scan_queue.push(*registered_scan_msg);
}

void MultiTransformNode::WayPointCallBack(
  const geometry_msgs::msg::PointStamped::ConstSharedPtr way_point_msg)
{
  std::shared_ptr<geometry_msgs::msg::PointStamped> local_point(
    new geometry_msgs::msg::PointStamped());
  local_point = std::make_shared<geometry_msgs::msg::PointStamped>(tf_buffer_->transform(
    *way_point_msg, "robot_" + std::to_string(robot_id) + "/map", tf2::durationFromSec(10.0)));
  local_way_point_pub_->publish(*local_point);
}

// void MultiTransformNode::TerrainMapCallBack(
//   const sensor_msgs::msg::PointCloud2::ConstSharedPtr terrain_map_msg)
// {
//   pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_tmp(new pcl::PointCloud<pcl::PointXYZI>());
//   pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_result(new pcl::PointCloud<pcl::PointXYZI>());
//   pcl::fromROSMsg(*terrain_map_msg, *pointcloud_tmp);
//   try{
//     pcl::transformPointCloud(*pointcloud_tmp, *pointcloud_result, *fromIdMapToMap);
//   }catch(const tf2::TransformException& ex){
//     RCLCPP_INFO(this->get_logger(), "%s", ex.what());
//     return;
//   }
//   std::shared_ptr<sensor_msgs::msg::PointCloud2> totalTerrainCloud(
//     new sensor_msgs::msg::PointCloud2());
//   pcl::toROSMsg(*pointcloud_result, *totalTerrainCloud);
//   totalTerrainCloud->header.stamp = terrain_map_msg->header.stamp;
//   totalTerrainCloud->header.frame_id = "map";
//   total_terrain_map_pub_->publish(*totalTerrainCloud);
// }

// void MultiTransformNode::TerrainMapExtCallBack(
//   const sensor_msgs::msg::PointCloud2::ConstSharedPtr terrain_map_ext_msg)
// {
//   pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_tmp(new pcl::PointCloud<pcl::PointXYZI>());
//   pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_result(new pcl::PointCloud<pcl::PointXYZI>());
//   pcl::fromROSMsg(*terrain_map_ext_msg, *pointcloud_tmp);
//   try{
//     pcl::transformPointCloud(*pointcloud_tmp, *pointcloud_result, *fromIdMapToMap);
//   }catch(const tf2::TransformException& ex){
//     RCLCPP_INFO(this->get_logger(), "%s", ex.what());
//     return;
//   }
//   std::shared_ptr<sensor_msgs::msg::PointCloud2> totalTerrainExtCloud(
//     new sensor_msgs::msg::PointCloud2());
//   pcl::toROSMsg(*pointcloud_result, *totalTerrainExtCloud);
//   totalTerrainExtCloud->header.stamp = terrain_map_ext_msg->header.stamp;
//   totalTerrainExtCloud->header.frame_id = "map";
//   total_terrain_map_ext_pub_->publish(*totalTerrainExtCloud);
// }

// void MultiTransformNode::StateEstimationAtScanCallBack(
//   const nav_msgs::msg::Odometry::ConstSharedPtr state_estimation_at_scan_msg)
// {
//   std::shared_ptr<geometry_msgs::msg::PoseStamped> local_state(new
//   geometry_msgs::msg::PoseStamped);
//   local_state->set__pose(state_estimation_at_scan_msg->pose.pose);
//   local_state->header = state_estimation_at_scan_msg->header;
//   std::shared_ptr<geometry_msgs::msg::PoseStamped> total_state;
//   total_state = std::make_shared<geometry_msgs::msg::PoseStamped>(
//     tf_buffer_->transform(*local_state, "map", tf2::durationFromSec(10.0)));
//   std::shared_ptr<nav_msgs::msg::Odometry> total_state_estimation_at_scan_msg(
//     new nav_msgs::msg::Odometry(*state_estimation_at_scan_msg));
//   total_state_estimation_at_scan_msg->pose.set__pose(total_state->pose);
//   total_state_estimation_at_scan_msg->header.frame_id = "map";
//   total_state_estimation_at_scan_pub_->publish(*total_state_estimation_at_scan_msg);
// }

} // namespace multi_transform

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(multi_transform::MultiTransformNode)