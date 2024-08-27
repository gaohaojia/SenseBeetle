#include <arpa/inet.h>
#include <geometry_msgs/msg/detail/transform_stamped__struct.hpp>
#include <netinet/in.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rmw/qos_profiles.h>
#include <rmw/rmw.h>
#include <rmw/types.h>
#include <sensor_msgs/msg/detail/image__struct.hpp>
#include <sensor_msgs/msg/detail/point_cloud2__struct.hpp>
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
#include <vector>

#include "robot_communication/robot_communication.hpp"

#define MAX_PACKET_SIZE 64000
#define BUFFER_SIZE 65535
#define MAX_BUFFER_QUEUE_SIZE 5

namespace robot_communication
{
RobotCommunicationNode::RobotCommunicationNode(const rclcpp::NodeOptions & options)
  : Node("robot_communication", options)
{
  this->declare_parameter<int>("robot_id", 0);
  this->declare_parameter<int>("network_port", 12130);
  this->declare_parameter<std::string>("network_ip", "192.168.31.207");

  this->get_parameter("robot_id", robot_id);
  this->get_parameter("network_port", port);
  this->get_parameter("network_ip", ip);

  registered_scan_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "registered_scan",
    5,
    std::bind(&RobotCommunicationNode::RegisteredScanCallBack, this, std::placeholders::_1));
  way_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "way_point",
    2,
    std::bind(&RobotCommunicationNode::WayPointCallBack, this, std::placeholders::_1));
  realsense_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "camera/camera/color/image_raw",
    2,
    std::bind(&RobotCommunicationNode::RealsenseImageCallBack, this, std::placeholders::_1));
  // terrain_map_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
  //   "terrain_map",
  //   2,
  //   std::bind(&RobotCommunicationNode::TerrainMapCallBack, this, std::placeholders::_1));
  // terrain_map_ext_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
  //   "terrain_map_ext",
  //   2,
  //   std::bind(&RobotCommunicationNode::TerrainMapExtCallBack, this, std::placeholders::_1));
  // state_estimation_at_scan_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
  //   "state_estimation_at_scan",
  //   5,
  //   std::bind(&RobotCommunicationNode::StateEstimationAtScanCallBack, this,
  //   std::placeholders::_1));

  local_way_point_pub_ =
    this->create_publisher<geometry_msgs::msg::PointStamped>("local_way_point", 2);

  InitMapTF();
  InitClient();
}

RobotCommunicationNode::~RobotCommunicationNode()
{
  if (send_thread_.joinable()) {
    send_thread_.join();
  }
  if (recv_thread_.joinable()) {
    recv_thread_.join();
  }
  if (tf_update_thread_.joinable()){
    tf_update_thread_.join();
  }
  close(sockfd);
}

// initialize the tf from local_map to map
void RobotCommunicationNode::InitMapTF()
{
  std::string fromFrameRel = "local_map";
  std::string toFrameRel = "map";
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  geometry_msgs::msg::TransformStamped transformStamped;
  try {
    transformStamped =
      tf_buffer_->lookupTransform(
        toFrameRel, fromFrameRel, tf2::TimePointZero, tf2::durationFromSec(10.0));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO(this->get_logger(),
                "Could not transform %s to %s: %s",
                toFrameRel.c_str(),
                fromFrameRel.c_str(),
                ex.what());
    return;
  }
  local_to_global = 
    tf2::transformToEigen(transformStamped.transform).matrix().cast<double>();
  tf_update_thread_ = std::thread(&RobotCommunicationNode::TFUpdateThread, this);
}

// initialize the socket client
void RobotCommunicationNode::InitClient()
{
  if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
    RCLCPP_ERROR(this->get_logger(), "Socket creation failed!");
    return;
  }

  memset(&server_addr, 0, sizeof(server_addr));

  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(port);
  server_addr.sin_addr.s_addr = inet_addr(ip.c_str());

  send_thread_ = std::thread(&RobotCommunicationNode::NetworkSendThread, this);
  recv_thread_ = std::thread(&RobotCommunicationNode::NetworkRecvThread, this);
  RCLCPP_INFO(this->get_logger(), "Client start! Target ip: %s, port: %d", ip.c_str(), port);
}

void RobotCommunicationNode::WayPointCallBack(
  const geometry_msgs::msg::PointStamped::ConstSharedPtr way_point_msg)
{
  geometry_msgs::msg::PointStamped local_point = 
    tf_buffer_->transform(*way_point_msg, "local_map", tf2::durationFromSec(10.0));
  local_way_point_pub_->publish(local_point);
}

void RobotCommunicationNode::RegisteredScanCallBack(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr registered_scan_msg)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_tmp(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_result(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::fromROSMsg(*registered_scan_msg, *pointcloud_tmp);
  try {
    pcl::transformPointCloud(*pointcloud_tmp, *pointcloud_result, local_to_global);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO(this->get_logger(), "%s", ex.what());
    return;
  }
  sensor_msgs::msg::PointCloud2 totalRegisteredScan;
  pcl::toROSMsg(*pointcloud_result, totalRegisteredScan);
  totalRegisteredScan.header.stamp = registered_scan_msg->header.stamp;
  totalRegisteredScan.header.frame_id = "map";

  std::vector<uint8_t> data_buffer =
    SerializeMsg<sensor_msgs::msg::PointCloud2>(totalRegisteredScan);
  PrepareBuffer(data_buffer, 0);
}

void RobotCommunicationNode::RealsenseImageCallBack(
  const sensor_msgs::msg::Image::ConstSharedPtr image_msg)
{
  std::vector<uint8_t> data_buffer = SerializeMsg<sensor_msgs::msg::Image>(*image_msg);
  PrepareBuffer(data_buffer, 1);
}

void RobotCommunicationNode::TFUpdateThread()
{
  geometry_msgs::msg::TransformStamped transformStamped;
  std::vector<uint8_t> data_buffer;
  while (rclcpp::ok()){
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    try {
      transformStamped = tf_buffer_->lookupTransform(
          "map", "vehicle", tf2::TimePointZero, tf2::durationFromSec(10.0));
    } catch (...) {
      continue;
    }
    transformStamped.child_frame_id = "robot_" + std::to_string(robot_id) + "/vehicle";
    data_buffer = SerializeMsg<geometry_msgs::msg::TransformStamped>(transformStamped);
    PrepareBuffer(data_buffer, 2);
  }
}

void RobotCommunicationNode::NetworkSendThread()
{
  while (rclcpp::ok()) {
    if (buffer_queue.empty()) {
      std::this_thread::sleep_for(std::chrono::nanoseconds(10));
      continue;
    }
    send_buffer s_buffer = buffer_queue.front();
    buffer_queue.pop();
    if (sendto(sockfd,
               s_buffer.buffer.data(),
               s_buffer.buffer.size(),
               0,
               (struct sockaddr *)&server_addr,
               sizeof(server_addr)) < 0) {
      RCLCPP_ERROR(this->get_logger(), "Send failed!");
    }
  }
}

void RobotCommunicationNode::NetworkRecvThread()
{
  int n, len = sizeof(server_addr);
  int packet_idx = 0;
  int packet_type = -1;
  std::vector<uint8_t> buffer;
  while (rclcpp::ok()) {
    std::vector<uint8_t> buffer_tmp(BUFFER_SIZE);
    n = recvfrom(sockfd,
                 buffer_tmp.data(),
                 BUFFER_SIZE,
                 MSG_WAITALL,
                 (struct sockaddr *)&server_addr,
                 (socklen_t *)&len);
    if (n < 0) {
      continue;
    }
    buffer_tmp.resize(n);
    uint8_t id, type, max_idx;
    uint16_t idx;
    std::memcpy(&id, buffer_tmp.data(), sizeof(id));
    std::memcpy(&type, buffer_tmp.data() + sizeof(uint8_t), sizeof(type));
    std::memcpy(&idx, buffer_tmp.data() + sizeof(uint16_t), sizeof(idx));
    std::memcpy(&max_idx, buffer_tmp.data() + sizeof(uint32_t), sizeof(max_idx));

    if (packet_type == -1) {
      packet_type = type;
    } else if (packet_type < type) {
      continue;
    } else if (packet_type > type) {
      packet_type = type;
      packet_idx = 0;
      buffer = std::vector<uint8_t>(0);
    }
    if (idx == 0) {
      packet_idx = 0;
      buffer = std::vector<uint8_t>(0);
    } else if (packet_idx != idx) {
      packet_idx = 0;
      packet_type = -1;
      buffer = std::vector<uint8_t>(0);
      continue;
    }

    packet_idx++;
    if (packet_idx == 1) {
      buffer.insert(
        buffer.begin(), buffer_tmp.begin() + sizeof(uint32_t) + sizeof(uint8_t), buffer_tmp.end());
    } else {
      buffer.insert(
        buffer.end(), buffer_tmp.begin() + sizeof(uint32_t) + sizeof(uint8_t), buffer_tmp.end());
    }

    if (packet_idx != max_idx) {
      continue;
    }
    try {
      if (type == 0) { // Way Point
        geometry_msgs::msg::PointStamped way_point =
            DeserializeMsg<geometry_msgs::msg::PointStamped>(buffer);
        WayPointCallBack(std::make_shared<geometry_msgs::msg::PointStamped>(way_point));
      }
    } catch (...) {
    }

    packet_idx = 0;
    packet_type = -1;
    buffer = std::vector<uint8_t>(0);
  }
}

void RobotCommunicationNode::PrepareBuffer(const std::vector<uint8_t> & data_buffer,
                                           const int msg_type)
{
  const int total_packet = (data_buffer.size() + MAX_PACKET_SIZE - 1) / MAX_PACKET_SIZE;
  for (int i = 0; i < total_packet; i++) {
    uint8_t id = robot_id;
    uint8_t type = msg_type;
    uint16_t idx = i;
    uint8_t max_idx = total_packet;
    std::vector<uint8_t> header(sizeof(uint32_t) + sizeof(uint8_t));
    std::memcpy(header.data(), &id, sizeof(id));
    std::memcpy(header.data() + sizeof(uint8_t), &type, sizeof(type));
    std::memcpy(header.data() + sizeof(uint16_t), &idx, sizeof(idx));
    std::memcpy(header.data() + sizeof(uint32_t), &max_idx, sizeof(max_idx));
    std::vector<uint8_t> packet;
    packet.insert(packet.end(), header.begin(), header.end());
    packet.insert(packet.end(),
                  data_buffer.begin() + i * MAX_PACKET_SIZE,
                  i == total_packet - 1 ? data_buffer.end()
                                        : data_buffer.begin() + (i + 1) * MAX_PACKET_SIZE);
    send_buffer s_buffer;
    s_buffer.buffer = packet;
    if (buffer_queue.size() >= MAX_BUFFER_QUEUE_SIZE) {
      buffer_queue.pop();
    }
    buffer_queue.push(s_buffer);
  }
}

// Serialization
template <class T> std::vector<uint8_t> RobotCommunicationNode::SerializeMsg(const T & msg)
{
  rclcpp::SerializedMessage serialized_msg;
  rclcpp::Serialization<T> serializer;
  serializer.serialize_message(&msg, &serialized_msg);

  std::vector<uint8_t> buffer_tmp(serialized_msg.size());
  std::memcpy(
    buffer_tmp.data(), serialized_msg.get_rcl_serialized_message().buffer, serialized_msg.size());

  return buffer_tmp;
}

// Deserialization
template <class T> T RobotCommunicationNode::DeserializeMsg(const std::vector<uint8_t> & data)
{
  rclcpp::SerializedMessage serialized_msg;
  rclcpp::Serialization<T> serializer;

  serialized_msg.reserve(data.size());
  std::memcpy(serialized_msg.get_rcl_serialized_message().buffer, data.data(), data.size());
  serialized_msg.get_rcl_serialized_message().buffer_length = data.size();

  T msg;
  serializer.deserialize_message(&serialized_msg, &msg);

  return msg;
}

// void RobotCommunicationNode::TerrainMapCallBack(
//   const sensor_msgs::msg::PointCloud2::ConstSharedPtr terrain_map_msg)
// {
//   pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_tmp(new
//   pcl::PointCloud<pcl::PointXYZI>()); pcl::PointCloud<pcl::PointXYZI>::Ptr
//   pointcloud_result(new pcl::PointCloud<pcl::PointXYZI>());
//   pcl::fromROSMsg(*terrain_map_msg, *pointcloud_tmp);
//   try{
//     pcl::transformPointCloud(*pointcloud_tmp, *pointcloud_result,
//     *fromIdMapToMap);
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

// void RobotCommunicationNode::TerrainMapExtCallBack(
//   const sensor_msgs::msg::PointCloud2::ConstSharedPtr terrain_map_ext_msg)
// {
//   pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_tmp(new
//   pcl::PointCloud<pcl::PointXYZI>()); pcl::PointCloud<pcl::PointXYZI>::Ptr
//   pointcloud_result(new pcl::PointCloud<pcl::PointXYZI>());
//   pcl::fromROSMsg(*terrain_map_ext_msg, *pointcloud_tmp);
//   try{
//     pcl::transformPointCloud(*pointcloud_tmp, *pointcloud_result,
//     *fromIdMapToMap);
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

// void RobotCommunicationNode::StateEstimationAtScanCallBack(
//   const nav_msgs::msg::Odometry::ConstSharedPtr state_estimation_at_scan_msg)
// {
//   std::shared_ptr<geometry_msgs::msg::PoseStamped> local_state(new
//   geometry_msgs::msg::PoseStamped);
//   local_state->set__pose(state_estimation_at_scan_msg->pose.pose);
//   local_state->header = state_estimation_at_scan_msg->header;
//   std::shared_ptr<geometry_msgs::msg::PoseStamped> total_state;
//   total_state = std::make_shared<geometry_msgs::msg::PoseStamped>(
//     tf_buffer_->transform(*local_state, "map", tf2::durationFromSec(10.0)));
//   std::shared_ptr<nav_msgs::msg::Odometry>
//   total_state_estimation_at_scan_msg(
//     new nav_msgs::msg::Odometry(*state_estimation_at_scan_msg));
//   total_state_estimation_at_scan_msg->pose.set__pose(total_state->pose);
//   total_state_estimation_at_scan_msg->header.frame_id = "map";
//   total_state_estimation_at_scan_pub_->publish(*total_state_estimation_at_scan_msg);
// }

} // namespace robot_communication

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(robot_communication::RobotCommunicationNode)