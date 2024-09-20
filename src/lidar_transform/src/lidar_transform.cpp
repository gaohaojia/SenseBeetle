#include <chrono>
#include <cstddef>
#include <memory>

#include "lidar_transform/lidar_transform.hpp"

namespace lidar_transform {
LidarTransform::LidarTransform(const rclcpp::NodeOptions& options)
    : Node("lidar_transform", options) {
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  while (rclcpp::ok()) {
    try {
      geometry_msgs::msg::TransformStamped transform_stamped =
        tf_buffer_->lookupTransform("base_link", "lidar", tf2::TimePointZero);

      tf2::fromMsg(transform_stamped.transform, frame_transform);
      break;
    } catch (tf2::TransformException& ex) {
      RCLCPP_WARN(this->get_logger(), "Could not transform IMU data: %s",
                  ex.what());
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "livox/imu", 10,
    std::bind(&LidarTransform::imu_callback, this, std::placeholders::_1));
  lidar_sub_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
    "livox/lidar", 10,
    std::bind(&LidarTransform::lidar_callback, this, std::placeholders::_1));

  imu_pub_ =
    this->create_publisher<sensor_msgs::msg::Imu>("livox/imu_transformed", 10);
  lidar_pub_ = this->create_publisher<livox_ros_driver2::msg::CustomMsg>(
    "livox/lidar_transformed", 10);
}

void LidarTransform::imu_callback(
  const sensor_msgs::msg::Imu::SharedPtr imu_msg) {
  tf2::Vector3 linear_accel(imu_msg->linear_acceleration.x,
                            imu_msg->linear_acceleration.y,
                            imu_msg->linear_acceleration.z);
  tf2::Vector3 angular_vel(imu_msg->angular_velocity.x,
                           imu_msg->angular_velocity.y,
                           imu_msg->angular_velocity.z);

  tf2::Vector3 transformed_accel = frame_transform.getBasis() * linear_accel;
  tf2::Vector3 transformed_angular_vel =
    frame_transform.getBasis() * angular_vel;

  sensor_msgs::msg::Imu imu_transformed = *imu_msg;
  imu_transformed.linear_acceleration.x = transformed_accel.getX();
  imu_transformed.linear_acceleration.y = transformed_accel.getY();
  imu_transformed.linear_acceleration.z = transformed_accel.getZ();
  imu_transformed.angular_velocity.x = transformed_angular_vel.getX();
  imu_transformed.angular_velocity.y = transformed_angular_vel.getY();
  imu_transformed.angular_velocity.z = transformed_angular_vel.getZ();

  imu_pub_->publish(imu_transformed);
}

void LidarTransform::lidar_callback(
  const livox_ros_driver2::msg::CustomMsg::SharedPtr lidar_msg) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(
    new pcl::PointCloud<pcl::PointXYZ>);
  for (size_t i = 0; i < lidar_msg->point_num; ++i) {
    pcl::PointXYZ point;
    point.x = lidar_msg->points[i].x;
    point.y = lidar_msg->points[i].y;
    point.z = lidar_msg->points[i].z;
    pcl_cloud->points.push_back(point);
  }
  pcl_cloud->width = pcl_cloud->points.size();
  pcl_cloud->height = 1;

  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(
    new pcl::PointCloud<pcl::PointXYZ>);
  pcl_ros::transformPointCloud(*pcl_cloud, *transformed_cloud, frame_transform);

  for (size_t i = 0; i < transformed_cloud->points.size(); ++i) {
    lidar_msg->points[i].x = transformed_cloud->points[i].x;
    lidar_msg->points[i].y = transformed_cloud->points[i].y;
    lidar_msg->points[i].z = transformed_cloud->points[i].z;
  }

  lidar_pub_->publish(*lidar_msg);
}

}  // namespace lidar_transform

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(lidar_transform::LidarTransform)