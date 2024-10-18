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
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <rclcpp/logging.hpp>
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
#define MAX_BUFFER_QUEUE_SIZE 256

#define LOCAL_MAP_FRAME "local_map"
#define MAP_FRAME "map"
#define CAMERA_FRAME "camera_depth_optical_frame"
#define BASE_FRAME "base_link"

namespace robot_communication {
RobotCommunicationNode::RobotCommunicationNode(
  const rclcpp::NodeOptions& options)
    : Node("robot_communication", options) {
  this->declare_parameter<int>("robot_id", 0);
  this->declare_parameter<int>("network_port", 12130);
  this->declare_parameter<std::string>("network_ip", "192.168.31.207");

  this->get_parameter("robot_id", robot_id);
  this->get_parameter("network_port", port);
  this->get_parameter("network_ip", ip);

  registered_scan_sub_ =
    this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "cloud_registered", 5,
      std::bind(&RobotCommunicationNode::RegisteredScanCallBack, this,
                std::placeholders::_1));
  way_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "way_point", 2,
    std::bind(&RobotCommunicationNode::WayPointCallBack, this,
              std::placeholders::_1));
  realsense_pointcloud_sub_ =
    this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "camera/camera/depth/color/points", 2,
      std::bind(&RobotCommunicationNode::RealsensePointCallBack, this,
                std::placeholders::_1));
  odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "Odometry", 2,
    std::bind(&RobotCommunicationNode::OdometryCallBack, this,
              std::placeholders::_1));

  local_way_point_pub_ =
    this->create_publisher<geometry_msgs::msg::PointStamped>("local_way_point",
                                                             2);

  InitMapTF();
  InitClient();
}

RobotCommunicationNode::~RobotCommunicationNode() {
  if (send_thread_.joinable()) {
    send_thread_.join();
  }
  if (recv_thread_.joinable()) {
    recv_thread_.join();
  }
  if (parse_buffer_thread_.joinable()) {
    parse_buffer_thread_.join();
  }
  if (tf_update_thread_.joinable()) {
    tf_update_thread_.join();
  }
  close(sockfd);
}

// initialize the tf from local_map to map
void RobotCommunicationNode::InitMapTF() {
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  geometry_msgs::msg::TransformStamped local_to_global_transform,
    camera_to_base_transform;
  while (rclcpp::ok()) {
    try {
      local_to_global_transform = tf_buffer_->lookupTransform(
        MAP_FRAME, LOCAL_MAP_FRAME, tf2::TimePointZero,
        tf2::durationFromSec(5.0));
      local_to_global_matrix =
        tf2::transformToEigen(local_to_global_transform.transform)
          .matrix()
          .cast<double>();
      break;
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
    }
  }
  try {
    camera_to_base_transform = tf_buffer_->lookupTransform(
      BASE_FRAME, CAMERA_FRAME, tf2::TimePointZero, tf2::durationFromSec(5.0));
    camera_to_base_matrix =
      tf2::transformToEigen(camera_to_base_transform.transform)
        .matrix()
        .cast<double>();
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN(this->get_logger(), "Could not transform camera!");
  }
  tf_update_thread_ =
    std::thread(&RobotCommunicationNode::TFUpdateThread, this);
}

// initialize the socket client
void RobotCommunicationNode::InitClient() {
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
  parse_buffer_thread_ =
    std::thread(&RobotCommunicationNode::ParseBufferThread, this);

  RCLCPP_INFO(this->get_logger(), "Client start! Target ip: %s, port: %d",
              ip.c_str(), port);
}

void RobotCommunicationNode::WayPointCallBack(
  const geometry_msgs::msg::PointStamped::ConstSharedPtr way_point_msg) {
  geometry_msgs::msg::PointStamped local_point = tf_buffer_->transform(
    *way_point_msg, LOCAL_MAP_FRAME, tf2::durationFromSec(10.0));
  local_way_point_pub_->publish(local_point);
}

void RobotCommunicationNode::RegisteredScanCallBack(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr registered_scan_msg) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_tmp(
    new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_result(
    new pcl::PointCloud<pcl::PointXYZI>());
  pcl::fromROSMsg(*registered_scan_msg, *pointcloud_tmp);

  try {
    pcl::transformPointCloud(*pointcloud_tmp, *pointcloud_result,
                             local_to_global_matrix);
  } catch (const tf2::TransformException& ex) {
    RCLCPP_INFO(this->get_logger(), "%s", ex.what());
    return;
  }
  sensor_msgs::msg::PointCloud2 totalRegisteredScan;
  pcl::toROSMsg(*pointcloud_result, totalRegisteredScan);
  totalRegisteredScan.header.stamp = registered_scan_msg->header.stamp;
  totalRegisteredScan.header.frame_id = MAP_FRAME;

  std::vector<uint8_t> data_buffer =
    SerializeMsg<sensor_msgs::msg::PointCloud2>(totalRegisteredScan);
  SendBuffer prepare_buffer = {robot_id, data_buffer, 0};
  PrepareBuffer(prepare_buffer);
}

void RobotCommunicationNode::RealsensePointCallBack(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr
    realsense_pointcloud_msg) {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_tmp(
    new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_tmp2(
    new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_result(
    new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::fromROSMsg(*realsense_pointcloud_msg, *pointcloud_tmp);

  try {
    pcl::transformPointCloud(*pointcloud_tmp, *pointcloud_tmp2,
                             camera_to_base_matrix);
    pcl::transformPointCloud(*pointcloud_tmp2, *pointcloud_result,
                             odom_to_local_matrix);
  } catch (const tf2::TransformException& ex) {
    RCLCPP_INFO(this->get_logger(), "%s", ex.what());
    return;
  }

  sensor_msgs::msg::PointCloud2 totalRegisteredScan;
  pcl::toROSMsg(*pointcloud_result, totalRegisteredScan);
  totalRegisteredScan.header.stamp = realsense_pointcloud_msg->header.stamp;
  totalRegisteredScan.header.frame_id = MAP_FRAME;

  std::vector<uint8_t> data_buffer =
    SerializeMsg<sensor_msgs::msg::PointCloud2>(totalRegisteredScan);
  SendBuffer prepare_buffer = {robot_id, data_buffer, 1};
  PrepareBuffer(prepare_buffer);
}

void RobotCommunicationNode::OdometryCallBack(
  const nav_msgs::msg::Odometry::ConstSharedPtr odometry_msg) {
  geometry_msgs::msg::TransformStamped odom_to_local_transform;
  odom_to_local_transform.transform.rotation =
    odometry_msg->pose.pose.orientation;
  odom_to_local_transform.transform.translation.x =
    odometry_msg->pose.pose.position.x;
  odom_to_local_transform.transform.translation.y =
    odometry_msg->pose.pose.position.y;
  odom_to_local_transform.transform.translation.z =
    odometry_msg->pose.pose.position.z;

  odom_to_local_matrix =
    tf2::transformToEigen(odom_to_local_transform.transform)
      .matrix()
      .cast<double>();
}

void RobotCommunicationNode::TFUpdateThread() {
  while (rclcpp::ok()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    Eigen::Matrix4d odom_to_map_matrix =
      odom_to_local_matrix * local_to_global_matrix;
    Eigen::Affine3d tmp(odom_to_map_matrix);
    geometry_msgs::msg::TransformStamped transformStamped =
      tf2::eigenToTransform(tmp);
    transformStamped.header.frame_id = MAP_FRAME;
    transformStamped.child_frame_id =
      "robot_" + std::to_string(robot_id) + "/base_link";
    transformStamped.header.stamp = this->get_clock()->now();
    std::vector<uint8_t> data_buffer =
      SerializeMsg<geometry_msgs::msg::TransformStamped>(transformStamped);
    SendBuffer prepare_buffer = {robot_id, data_buffer, 2};
    PrepareBuffer(prepare_buffer);
  }
}

void RobotCommunicationNode::NetworkSendThread() {
  while (rclcpp::ok()) {
    if (send_buffer_queue.empty()) {
      std::this_thread::sleep_for(std::chrono::nanoseconds(10));
      continue;
    }
    SendBuffer s_buffer = send_buffer_queue.front();
    send_buffer_queue.pop();
    if (sendto(sockfd, s_buffer.buffer.data(), s_buffer.buffer.size(), 0,
               (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
      RCLCPP_ERROR(this->get_logger(), "Send failed!");
    }
  }
}

void RobotCommunicationNode::NetworkRecvThread() {
  int n, len = sizeof(server_addr);
  while (rclcpp::ok()) {
    std::vector<uint8_t> buffer_tmp(BUFFER_SIZE);
    n = recvfrom(sockfd, buffer_tmp.data(), BUFFER_SIZE, MSG_WAITFORONE,
                 (struct sockaddr*)&server_addr, (socklen_t*)&len);
    if (n < 0) {
      continue;
    }
    buffer_tmp.resize(n);
    if (recv_buffer_queue.size() >= MAX_BUFFER_QUEUE_SIZE) {
      continue;
    }
    recv_buffer_queue.push(buffer_tmp);
  }
}

void RobotCommunicationNode::PrepareBuffer(const SendBuffer& prepare_buffer) {
  const int total_packet =
    (prepare_buffer.buffer.size() + MAX_PACKET_SIZE - 1) / MAX_PACKET_SIZE;
  for (int i = 0; i < total_packet; i++) {
    uint8_t id = prepare_buffer.id;
    uint8_t type = prepare_buffer.msg_type;
    uint16_t idx = i;
    uint8_t max_idx = total_packet;
    std::vector<uint8_t> header(sizeof(uint32_t) + sizeof(uint8_t));
    std::memcpy(header.data(), &id, sizeof(id));
    std::memcpy(header.data() + sizeof(uint8_t), &type, sizeof(type));
    std::memcpy(header.data() + sizeof(uint16_t), &idx, sizeof(idx));
    std::memcpy(header.data() + sizeof(uint32_t), &max_idx, sizeof(max_idx));
    std::vector<uint8_t> packet;
    packet.insert(packet.end(), header.begin(), header.end());
    packet.insert(
      packet.end(), prepare_buffer.buffer.begin() + i * MAX_PACKET_SIZE,
      i == total_packet - 1
        ? prepare_buffer.buffer.end()
        : prepare_buffer.buffer.begin() + (i + 1) * MAX_PACKET_SIZE);
    SendBuffer s_buffer;
    s_buffer.buffer = packet;
    send_buffer_queue.push(s_buffer);
  }
}

void RobotCommunicationNode::ParseBufferThread() {
  int packet_idx = 0;
  int packet_type = -1;
  std::vector<uint8_t> buffer;
  while (rclcpp::ok()) {
    if (recv_buffer_queue.empty()) {
      std::this_thread::sleep_for(std::chrono::nanoseconds(10));
      continue;
    }
    std::vector<uint8_t> buffer_tmp = recv_buffer_queue.front();
    recv_buffer_queue.pop();

    uint8_t id, type, max_idx;
    uint16_t idx;
    std::memcpy(&id, buffer_tmp.data(), sizeof(id));
    std::memcpy(&type, buffer_tmp.data() + sizeof(uint8_t), sizeof(type));
    std::memcpy(&idx, buffer_tmp.data() + sizeof(uint16_t), sizeof(idx));
    std::memcpy(&max_idx, buffer_tmp.data() + sizeof(uint32_t),
                sizeof(max_idx));

    if (packet_type == -1) {
      packet_type = type;
    } else if (packet_type != type) {
      packet_idx = 0;
      packet_type = -1;
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
      buffer.insert(buffer.begin(),
                    buffer_tmp.begin() + sizeof(uint32_t) + sizeof(uint8_t),
                    buffer_tmp.end());
    } else {
      buffer.insert(buffer.end(),
                    buffer_tmp.begin() + sizeof(uint32_t) + sizeof(uint8_t),
                    buffer_tmp.end());
    }

    if (packet_idx != max_idx) {
      continue;
    }
    try {
      if (type == 0) {  // Way Point
        geometry_msgs::msg::PointStamped way_point =
          DeserializeMsg<geometry_msgs::msg::PointStamped>(buffer);
        WayPointCallBack(
          std::make_shared<geometry_msgs::msg::PointStamped>(way_point));
      }
    } catch (...) {
    }

    packet_idx = 0;
    packet_type = -1;
    buffer = std::vector<uint8_t>(0);
  }
}

// Serialization
template <class T>
std::vector<uint8_t> RobotCommunicationNode::SerializeMsg(const T& msg) {
  rclcpp::SerializedMessage serialized_msg;
  rclcpp::Serialization<T> serializer;
  serializer.serialize_message(&msg, &serialized_msg);

  std::vector<uint8_t> buffer_tmp(serialized_msg.size());
  std::memcpy(buffer_tmp.data(),
              serialized_msg.get_rcl_serialized_message().buffer,
              serialized_msg.size());

  return buffer_tmp;
}

// Deserialization
template <class T>
T RobotCommunicationNode::DeserializeMsg(const std::vector<uint8_t>& data) {
  rclcpp::SerializedMessage serialized_msg;
  rclcpp::Serialization<T> serializer;

  serialized_msg.reserve(data.size());
  std::memcpy(serialized_msg.get_rcl_serialized_message().buffer, data.data(),
              data.size());
  serialized_msg.get_rcl_serialized_message().buffer_length = data.size();

  T msg;
  serializer.deserialize_message(&serialized_msg, &msg);

  return msg;
}

}  // namespace robot_communication

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(robot_communication::RobotCommunicationNode)