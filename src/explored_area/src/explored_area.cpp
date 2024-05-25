#include "explored_area/explored_area.hpp"
#include <functional>
#include <memory>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace explored_area
{
ExploredAreaNode::ExploredAreaNode(const rclcpp::NodeOptions & options)
  : Node("explored_area", options)
{
  this->declare_parameter<int>("robot_id", 0);
  this->get_parameter("robot_id", robot_id);

  registered_scan_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "registered_scan",
    5,
    std::bind(&ExploredAreaNode::RegisteredScanCallBack, this, std::placeholders::_1));

  explored_area_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("explored_areas", 5);
  explored_volume_pub_ = this->create_publisher<std_msgs::msg::Float32>("explored_volume", 5);
  traveling_dis_pub_ = this->create_publisher<std_msgs::msg::Float32>("traveling_distance", 5);

  exploredVolumeCloud =
    std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  exploredVolumeCloud2 =
    std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  exploredAreaCloud =
    std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  exploredAreaCloud2 =
    std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

  exploredVolumeDwzFilter.setLeafSize(
    exploredVolumeVoxelSize, exploredVolumeVoxelSize, exploredVolumeVoxelSize);
  exploredAreaDwzFilter.setLeafSize(
    exploredAreaVoxelSize, exploredAreaVoxelSize, exploredAreaVoxelSize);
}

void ExploredAreaNode::RegisteredScanCallBack(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr registered_scan_msg)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::fromROSMsg(*registered_scan_msg, *laserCloud);

  *exploredVolumeCloud += *laserCloud;

  // exploredVolumeCloud2->clear();
  // exploredVolumeDwzFilter.setInputCloud(exploredVolumeCloud);
  // exploredVolumeDwzFilter.filter(*exploredVolumeCloud2);

  pcl::PointCloud<pcl::PointXYZI>::Ptr tempCloud = exploredVolumeCloud;
  // exploredVolumeCloud = exploredVolumeCloud2;
  exploredVolumeCloud2 = tempCloud;

  exploredVolume = exploredVolumeVoxelSize * exploredVolumeVoxelSize * exploredVolumeVoxelSize *
                   exploredVolumeCloud->points.size();

  *exploredAreaCloud += *laserCloud;

  exploredAreaDisplayCount++;
  if (exploredAreaDisplayCount >= 5 * exploredAreaDisplayInterval) {
    // exploredAreaCloud2->clear();
    // exploredAreaDwzFilter.setInputCloud(exploredAreaCloud);
    // exploredAreaDwzFilter.filter(*exploredAreaCloud2);

    tempCloud = exploredAreaCloud;
    // exploredAreaCloud = exploredAreaCloud2;
    exploredAreaCloud2 = tempCloud;

    sensor_msgs::msg::PointCloud2 exploredArea2;
    pcl::toROSMsg(*exploredAreaCloud, exploredArea2);
    exploredArea2.header.stamp = registered_scan_msg->header.stamp;
    exploredArea2.header.frame_id = "robot_" + std::to_string(robot_id) + "/map";
    explored_area_pub_->publish(exploredArea2);

    exploredAreaDisplayCount = 0;
  }

  // fprintf(metricFilePtr, "%f %f %f %f\n", exploredVolume, travelingDis, runtime, timeDuration);

  std_msgs::msg::Float32 exploredVolumeMsg;
  exploredVolumeMsg.data = exploredVolume;
  explored_volume_pub_->publish(exploredVolumeMsg);

  std_msgs::msg::Float32 travelingDisMsg;
  travelingDisMsg.data = travelingDis;
  traveling_dis_pub_->publish(travelingDisMsg);
}
} // namespace explored_area

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(explored_area::ExploredAreaNode)