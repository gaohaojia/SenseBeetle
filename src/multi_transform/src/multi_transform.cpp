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

#include "multi_transform/multi_transform.hpp"

#define MAX_PACKET_SIZE 64000
#define BUFFER_SIZE 65535

namespace multi_transform {
MultiTransformNode::MultiTransformNode(const rclcpp::NodeOptions &options)
    : Node("multi_transform", options) {
  this->declare_parameter<int>("robot_id", 0);
  this->declare_parameter<int>("network_port", 12130);
  this->declare_parameter<std::string>("network_ip", "192.168.31.207");

  this->get_parameter("robot_id", robot_id);
  this->get_parameter("network_port", port);
  this->get_parameter("network_ip", ip);

  registered_scan_sub_ =
      this->create_subscription<sensor_msgs::msg::PointCloud2>(
          "registered_scan", 5,
          std::bind(&MultiTransformNode::RegisteredScanCallBack, this,
                    std::placeholders::_1));
  way_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "way_point", 2,
      std::bind(&MultiTransformNode::WayPointCallBack, this,
                std::placeholders::_1));
  realsense_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "camera/camera/color/image_raw", 2,
      std::bind(&MultiTransformNode::RealsenseImageCallBack, this,
                std::placeholders::_1));
  // terrain_map_sub_ =
  // this->create_subscription<sensor_msgs::msg::PointCloud2>(
  //   "terrain_map",
  //   2,
  //   std::bind(&MultiTransformNode::TerrainMapCallBack, this,
  //   std::placeholders::_1));
  // terrain_map_ext_sub_ =
  // this->create_subscription<sensor_msgs::msg::PointCloud2>(
  //   "terrain_map_ext",
  //   2,
  //   std::bind(&MultiTransformNode::TerrainMapExtCallBack, this,
  //   std::placeholders::_1));
  // state_estimation_at_scan_sub_ =
  // this->create_subscription<nav_msgs::msg::Odometry>(
  //   "state_estimation_at_scan",
  //   5,
  //   std::bind(&MultiTransformNode::StateEstimationAtScanCallBack, this,
  //   std::placeholders::_1));

  local_way_point_pub_ =
      this->create_publisher<geometry_msgs::msg::PointStamped>(
          "local_way_point", 2);
  // total_terrain_map_pub_ =
  //   this->create_publisher<sensor_msgs::msg::PointCloud2>("total_terrain_map",
  //   2);
  // total_terrain_map_ext_pub_ =
  //   this->create_publisher<sensor_msgs::msg::PointCloud2>("total_terrain_map_ext",
  //   2);
  // total_state_estimation_at_scan_pub_ =
  //   this->create_publisher<nav_msgs::msg::Odometry>("total_state_estimation_at_scan",
  //   5);

  std::string fromFrameRel = "robot_" + std::to_string(robot_id) + "/map";
  std::string toFrameRel = "map";
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  std::shared_ptr<geometry_msgs::msg::TransformStamped> transformStamped;
  try {
    transformStamped = std::make_shared<geometry_msgs::msg::TransformStamped>(
        tf_buffer_->lookupTransform(toFrameRel, fromFrameRel,
                                    tf2::TimePointZero,
                                    tf2::durationFromSec(10.0)));
  } catch (const tf2::TransformException &ex) {
    RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s",
                toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
    return;
  }
  fromIdMapToMap = std::make_shared<Eigen::Matrix4d>(
      tf2::transformToEigen(transformStamped->transform)
          .matrix()
          .cast<double>());

  // UDP
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
  RCLCPP_INFO(this->get_logger(), "Client start! Target ip: %s, port: %d",
              ip.c_str(), port);
}

MultiTransformNode::~MultiTransformNode() {
  if (send_thread_.joinable()) {
    send_thread_.join();
  }
  if (recv_thread_.joinable()) {
    recv_thread_.join();
  }
  close(sockfd);
}

void MultiTransformNode::NetworkSendThread() {
  std::shared_ptr<std::vector<uint8_t>> data_buffer;
  while (rclcpp::ok()) {
    // PointCloud2
    if (!registered_scan_queue.empty()) {
      data_buffer = std::make_shared<std::vector<uint8_t>>(
          MultiTransformNode::SerializeMsg<sensor_msgs::msg::PointCloud2>(
              registered_scan_queue.front()));
      registered_scan_queue.pop();
      SendData(*data_buffer, 0);
    }

    // Image
    if (!realsense_image_queue.empty()) {
      data_buffer = std::make_shared<std::vector<uint8_t>>(
          MultiTransformNode::SerializeMsg<sensor_msgs::msg::Image>(
              realsense_image_queue.front()));
      realsense_image_queue.pop();
      SendData(*data_buffer, 1);
    }

    // Transform
    std::shared_ptr<geometry_msgs::msg::TransformStamped> transformStamped;
    try {
      transformStamped = std::make_shared<geometry_msgs::msg::TransformStamped>(
          tf_buffer_->lookupTransform(
              "map", "robot_" + std::to_string(robot_id) + "/vehicle",
              tf2::TimePointZero, tf2::durationFromSec(10.0)));
    } catch (...) {
      continue;
    }
    data_buffer = std::make_shared<std::vector<uint8_t>>(
        MultiTransformNode::SerializeMsg<geometry_msgs::msg::TransformStamped>(
            *transformStamped));
    SendData(*data_buffer, 2);
  }
}

void MultiTransformNode::NetworkRecvThread() {
  int n, len = sizeof(server_addr);
  int packet_idx = 0;
  int packet_type = -1;
  std::vector<uint8_t> buffer;
  while (rclcpp::ok()) {
    std::vector<uint8_t> buffer_tmp(BUFFER_SIZE);
    n = recvfrom(sockfd, buffer_tmp.data(), BUFFER_SIZE, MSG_WAITALL,
                 (struct sockaddr *)&server_addr, (socklen_t *)&len);
    if (n < 0) {
      continue;
    }
    buffer_tmp.resize(n);
    uint8_t id, type, max_idx;
    uint16_t idx;
    std::memcpy(&id, buffer_tmp.data(), sizeof(id));
    std::memcpy(&type, buffer_tmp.data() + sizeof(uint8_t), sizeof(type));
    std::memcpy(&idx, buffer_tmp.data() + sizeof(uint16_t), sizeof(idx));
    std::memcpy(&max_idx, buffer_tmp.data() + sizeof(uint32_t),
                sizeof(max_idx));

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
      if (type == 0) { // Way Point
        std::shared_ptr<geometry_msgs::msg::PointStamped> way_point =
            std::make_shared<geometry_msgs::msg::PointStamped>(
                MultiTransformNode::DeserializeMsg<
                    geometry_msgs::msg::PointStamped>(buffer));
        MultiTransformNode::WayPointCallBack(way_point);
      }
    } catch (...) {
    }

    packet_idx = 0;
    packet_type = -1;
    buffer = std::vector<uint8_t>(0);
  }
}

void MultiTransformNode::SendData(const std::vector<uint8_t> &data_buffer,
                                  const int msg_type) {
  const int total_packet =
      (data_buffer.size() + MAX_PACKET_SIZE - 1) / MAX_PACKET_SIZE;
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
    packet.insert(packet.end(), data_buffer.begin() + i * MAX_PACKET_SIZE,
                  i == total_packet - 1
                      ? data_buffer.end()
                      : data_buffer.begin() + (i + 1) * MAX_PACKET_SIZE);
    sendto(sockfd, packet.data(), packet.size(), MSG_CONFIRM,
           (const struct sockaddr *)&server_addr, sizeof(server_addr));
  }
  rclcpp::sleep_for(std::chrono::nanoseconds(100));
}

// Serialization
template <class T>
std::vector<uint8_t> MultiTransformNode::SerializeMsg(const T &msg) {
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
T MultiTransformNode::DeserializeMsg(const std::vector<uint8_t> &data) {
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

void MultiTransformNode::RegisteredScanCallBack(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr registered_scan_msg) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_tmp(
      new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_result(
      new pcl::PointCloud<pcl::PointXYZI>());
  pcl::fromROSMsg(*registered_scan_msg, *pointcloud_tmp);
  try {
    pcl::transformPointCloud(*pointcloud_tmp, *pointcloud_result,
                             *fromIdMapToMap);
  } catch (const tf2::TransformException &ex) {
    RCLCPP_INFO(this->get_logger(), "%s", ex.what());
    return;
  }
  std::shared_ptr<sensor_msgs::msg::PointCloud2> totalRegisteredScan(
      new sensor_msgs::msg::PointCloud2());
  pcl::toROSMsg(*pointcloud_result, *totalRegisteredScan);
  totalRegisteredScan->header.stamp = registered_scan_msg->header.stamp;
  totalRegisteredScan->header.frame_id = "map";
  if (registered_scan_queue.size() >= 2) {
    registered_scan_queue.pop();
  }
  registered_scan_queue.push(*totalRegisteredScan);
}

void MultiTransformNode::WayPointCallBack(
    const geometry_msgs::msg::PointStamped::ConstSharedPtr way_point_msg) {
  std::shared_ptr<geometry_msgs::msg::PointStamped> local_point(
      new geometry_msgs::msg::PointStamped());
  local_point =
      std::make_shared<geometry_msgs::msg::PointStamped>(tf_buffer_->transform(
          *way_point_msg, "robot_" + std::to_string(robot_id) + "/map",
          tf2::durationFromSec(10.0)));
  local_way_point_pub_->publish(*local_point);
}

void MultiTransformNode::RealsenseImageCallBack(
    const sensor_msgs::msg::Image::ConstSharedPtr image_msg) {
  if (realsense_image_queue.size() >= 2) {
    realsense_image_queue.pop();
  }
  realsense_image_queue.push(*image_msg);
}
// void MultiTransformNode::TerrainMapCallBack(
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

// void MultiTransformNode::TerrainMapExtCallBack(
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
//   std::shared_ptr<nav_msgs::msg::Odometry>
//   total_state_estimation_at_scan_msg(
//     new nav_msgs::msg::Odometry(*state_estimation_at_scan_msg));
//   total_state_estimation_at_scan_msg->pose.set__pose(total_state->pose);
//   total_state_estimation_at_scan_msg->header.frame_id = "map";
//   total_state_estimation_at_scan_pub_->publish(*total_state_estimation_at_scan_msg);
// }

} // namespace multi_transform

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(multi_transform::MultiTransformNode)