#ifndef EXPLORED_AREA
#define EXPLORED_AREA

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32.hpp>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/filters/voxel_grid.h>

namespace explored_area
{
class ExploredAreaNode : public rclcpp::Node
{
public:
  ExploredAreaNode(const rclcpp::NodeOptions & options);

private:
  int robot_id;
  int exploredAreaDisplayCount = 0;
  int exploredAreaDisplayInterval = 1;
  double exploredVolumeVoxelSize = 0.5;
  double exploredAreaVoxelSize = 0.3;
  float exploredVolume = 0;
  float travelingDis = 0;

  pcl::VoxelGrid<pcl::PointXYZI> exploredVolumeDwzFilter;
  pcl::VoxelGrid<pcl::PointXYZI> exploredAreaDwzFilter;
  
  pcl::PointCloud<pcl::PointXYZI>::Ptr exploredVolumeCloud;
  pcl::PointCloud<pcl::PointXYZI>::Ptr exploredVolumeCloud2;
  pcl::PointCloud<pcl::PointXYZI>::Ptr exploredAreaCloud;
  pcl::PointCloud<pcl::PointXYZI>::Ptr exploredAreaCloud2;

  void
  RegisteredScanCallBack(const sensor_msgs::msg::PointCloud2::ConstSharedPtr registered_scan_msg);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr registered_scan_sub_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr explored_area_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr explored_volume_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr traveling_dis_pub_;
};

} // namespace explored_area

#endif