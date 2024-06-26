#include <arpa/inet.h>
#include <netinet/in.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rmw/qos_profiles.h>
#include <rmw/rmw.h>
#include <rmw/types.h>
#include <sys/socket.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/convert.h>
#include <tf2/time.h>
#include <tf2/transform_datatypes.h>

#include <chrono>
#include <cstdint>
#include <cstring>
#include <functional>
#include <geometry_msgs/msg/detail/point_stamped__struct.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <rclcpp/qos.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rclcpp/utilities.hpp>
#include <string>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>

#include "multi_transform/multi_transform.hpp"

#define MAX_PACKET_SIZE 64000
#define BUFFER_SIZE 65535

namespace multi_transform
{
MultiTransformNode::MultiTransformNode(const rclcpp::NodeOptions & options)
  : Node("multi_transform", options)
{
  this->declare_parameter<int>("robot_id", 0);
  this->declare_parameter<int>("network_port", 12130);
  this->declare_parameter<std::string>("network_ip", "192.168.31.207");
  this->declare_parameter<double>("multiOffsetPositionX", 0.0);
  this->declare_parameter<double>("multiOffsetPositionY", 0.0);
  this->declare_parameter<double>("multiOffsetPositionZ", 0.0);
  this->declare_parameter<double>("multiOffsetRotateR", 0.0);
  this->declare_parameter<double>("multiOffsetRotateP", 0.0);
  this->declare_parameter<double>("multiOffsetRotateY", 0.0);

  this->get_parameter("robot_id", robot_id);
  this->get_parameter("network_port", port);
  this->get_parameter("network_ip", ip);
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
    "way_point",
    2,
    std::bind(&MultiTransformNode::WayPointCallBack, this, std::placeholders::_1));
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
    transformStamped = std::make_shared<geometry_msgs::msg::TransformStamped>(
      tf_buffer_->lookupTransform(toFrameRel,
                                  fromFrameRel,
                                  tf2::TimePointZero,
                                  tf2::durationFromSec(10.0)));
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

  // TCP网络通讯部分
  if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
    RCLCPP_ERROR(this->get_logger(), "Socket creation failed!");
    return;
  }

  memset(&server_addr, 0, sizeof(server_addr));

  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(port);
  server_addr.sin_addr.s_addr = inet_addr(ip.c_str());

  send_thread_ = std::thread(&MultiTransformNode::NetworkSendThread, this);
  recv_thread_ = std::thread(&MultiTransformNode::NetworkRecvThread, this);
  RCLCPP_INFO(this->get_logger(), "Client start at ip: %s, port: %d", ip.c_str(), port);
}

MultiTransformNode::~MultiTransformNode()
{
  if (send_thread_.joinable()) {
    send_thread_.join();
  }
  if (recv_thread_.joinable()) {
    recv_thread_.join();
  }
  close(sockfd);
}

void MultiTransformNode::NetworkSendThread()
{
  int len = sizeof(server_addr);
  while (rclcpp::ok()) {
    rclcpp::sleep_for(std::chrono::nanoseconds(100));
    if (!registered_scan_queue.empty()) {
      std::vector<uint8_t> data_buffer = MultiTransformNode::SerializePointCloud2(registered_scan_queue.front());
      registered_scan_queue.pop();
      // RCLCPP_INFO(this->get_logger(), "%ld", data_buffer.size());
      const int total_packet = (data_buffer.size() + MAX_PACKET_SIZE - 1) / MAX_PACKET_SIZE;
      for (int i = 0; i < total_packet; i++){
        uint8_t header1 = robot_id; // id
        uint8_t header2 = 0x00; // 数据类型
        uint16_t header3 = i; // packet 编号
        uint8_t header4 = total_packet;
        std::vector<uint8_t> header(sizeof(header1) + sizeof(header2) + sizeof(header3));
        std::memcpy(header.data(), &header1, sizeof(header1));
        std::memcpy(header.data() + sizeof(header1), &header2, sizeof(header2));
        std::memcpy(header.data() + sizeof(header1) + sizeof(header2), &header3, sizeof(header3));
        std::memcpy(header.data() + sizeof(header1) + sizeof(header2) + sizeof(header3), &header4, sizeof(header4));
        std::vector<uint8_t> packet(sizeof(header) + sizeof(data_buffer));
        packet.insert(packet.end(), header.begin(), header.end());
        packet.insert(packet.end(), data_buffer.begin(), data_buffer.end());
        RCLCPP_INFO(this->get_logger(), "%ld", packet.size());
        sendto(sockfd,
             packet.data(),
             packet.size(),
             MSG_CONFIRM,
             (const struct sockaddr *)&server_addr,
             len);
      }
    }
  }
}

void MultiTransformNode::NetworkRecvThread()
{
  int n, len = sizeof(server_addr);
  while (rclcpp::ok()) {
    std::vector<uint8_t> buffer(BUFFER_SIZE);
    n = recvfrom(sockfd,
                 buffer.data(),
                 BUFFER_SIZE,
                 0,
                 (struct sockaddr *)&server_addr,
                 (socklen_t *)&len);
    buffer.resize(n);

    RCLCPP_INFO(this->get_logger(), "%s", buffer.data());
  }
}

// PointCloud2 序列化
std::vector<uint8_t> MultiTransformNode::SerializePointCloud2(
  const sensor_msgs::msg::PointCloud2 & pointcloud2_msg)
{
  rclcpp::SerializedMessage serialized_msg;
  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serializer;
  serializer.serialize_message(&pointcloud2_msg, &serialized_msg);

  // 将序列化的消息复制到字节数组
  std::vector<uint8_t> buffer_tmp(serialized_msg.size());
  std::memcpy(buffer_tmp.data(),
              serialized_msg.get_rcl_serialized_message().buffer,
              serialized_msg.size());

  return buffer_tmp;
}

void MultiTransformNode::RegisteredScanCallBack(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr registered_scan_msg)
{
  if (registered_scan_queue.size() >= 5) {
    registered_scan_queue.pop();
  }
  registered_scan_queue.push(*registered_scan_msg);
}

void MultiTransformNode::WayPointCallBack(
  const geometry_msgs::msg::PointStamped::ConstSharedPtr way_point_msg)
{
  std::shared_ptr<geometry_msgs::msg::PointStamped> local_point(
    new geometry_msgs::msg::PointStamped());
  local_point = std::make_shared<geometry_msgs::msg::PointStamped>(
    tf_buffer_->transform(*way_point_msg,
                          "robot_" + std::to_string(robot_id) + "/map",
                          tf2::durationFromSec(10.0)));
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

}  // namespace multi_transform

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(multi_transform::MultiTransformNode)