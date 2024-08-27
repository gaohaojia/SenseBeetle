#ifndef ROBOT_COMMUNICATION
#define ROBOT_COMMUNICATION

#include <netinet/in.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/LinearMath/Transform.h>

#include <cstdint>
#include <geometry_msgs/msg/detail/transform_stamped__struct.hpp>
#include <memory>
#include <pcl/impl/point_types.hpp>
#include <queue>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/detail/image__struct.hpp>
#include <sensor_msgs/msg/detail/point_cloud2__struct.hpp>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2/transform_datatypes.h"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace robot_communication
{
class RobotCommunicationNode : public rclcpp::Node
{
public:
  RobotCommunicationNode(const rclcpp::NodeOptions & options);
  ~RobotCommunicationNode() override;

private:
  int port;
  std::string ip;
  int sockfd;
  struct sockaddr_in server_addr;

  int robot_id;

  std::thread send_thread_;
  std::thread recv_thread_;
  std::thread tf_update_thread_;
  std::thread prepare_buffer_thread_;
  std::thread parse_buffer_thread_;

  struct SendBuffer
  {
    std::vector<uint8_t> buffer;
  };
  std::queue<SendBuffer> buffer_queue;

  struct PrepareBuffer
  {
    int id;
    std::vector<uint8_t> buffer;
    int msg_type;
  };
  std::queue<PrepareBuffer> prepare_buffer_queue;
  std::queue<std::vector<uint8_t>> parse_buffer_queue;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  Eigen::Matrix4d local_to_global;

  void InitMapTF();
  void InitClient();

  void NetworkSendThread();
  void NetworkRecvThread();
  void TFUpdateThread();
  void PrepareBufferThread();
  void ParseBufferThread();

  void TerrainMapCallBack(const sensor_msgs::msg::PointCloud2::ConstSharedPtr terrain_map_msg);
  void
  TerrainMapExtCallBack(const sensor_msgs::msg::PointCloud2::ConstSharedPtr terrain_map_ext_msg);
  void
  RegisteredScanCallBack(const sensor_msgs::msg::PointCloud2::ConstSharedPtr registered_scan_msg);
  void StateEstimationAtScanCallBack(
    const nav_msgs::msg::Odometry::ConstSharedPtr state_estimation_at_scan_msg);
  void RealsenseImageCallBack(const sensor_msgs::msg::Image::ConstSharedPtr image_msg);

  void WayPointCallBack(const geometry_msgs::msg::PointStamped::ConstSharedPtr way_point_msg);

  template <class T> std::vector<uint8_t> SerializeMsg(const T & msg);
  template <class T> T DeserializeMsg(const std::vector<uint8_t> & data);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr terrain_map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr terrain_map_ext_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr registered_scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr state_estimation_at_scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr way_point_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr realsense_image_sub_;

  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr local_way_point_pub_;
};
} // namespace robot_communication

#endif