#include "lidar_transform/lidar_transform.hpp"
#include <chrono>
#include <cstddef>
#include <memory>

namespace lidar_transform
{
LidarTransform::LidarTransform(const rclcpp::NodeOptions & options)
  : Node("lidar_transform", options)
{
  this->declare_parameter<bool>("enable_pid", false);
  this->declare_parameter<double>("pid_kp", 0.0);
  this->declare_parameter<double>("pid_ki", 0.0);
  this->declare_parameter<double>("pid_kd", 0.0);

  this->get_parameter("enable_pid", enable_pid);
  this->get_parameter("pid_kp", pid_kp);
  this->get_parameter("pid_ki", pid_ki);  
  this->get_parameter("pid_kd", pid_kd);

  pid_current_velocity = geometry_msgs::msg::Twist();
  pid_target_velocity = geometry_msgs::msg::Twist();
  pid_integral = geometry_msgs::msg::Twist();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  while (rclcpp::ok()) {
    try {
      geometry_msgs::msg::TransformStamped transform_stamped =
        tf_buffer_->lookupTransform("lidar", "imu_link", tf2::TimePointZero);

      tf2::fromMsg(transform_stamped.transform, frame_transform);
      break;
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "Could not transform IMU data: %s", ex.what());
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "livox/imu", 10, std::bind(&LidarTransform::imu_callback, this, std::placeholders::_1));
  lidar_sub_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
    "livox/lidar", 10, std::bind(&LidarTransform::lidar_callback, this, std::placeholders::_1));
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    "cmd_vel", 10, std::bind(&LidarTransform::cmd_vel_callback, this, std::placeholders::_1));

  imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("livox/imu_transformed", 10);
  lidar_pub_ =
    this->create_publisher<livox_ros_driver2::msg::CustomMsg>("livox/lidar_transformed", 10);
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel_pid", 10);

  if (enable_pid) {
    RCLCPP_INFO(this->get_logger(), "PID controller enabled!");
  }
}

void LidarTransform::imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
{
  tf2::Vector3 linear_accel(
    imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z);
  tf2::Vector3 angular_vel(
    imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z);

  tf2::Vector3 transformed_accel = frame_transform.getBasis() * linear_accel;
  tf2::Vector3 transformed_angular_vel = frame_transform.getBasis() * angular_vel;

  sensor_msgs::msg::Imu imu_transformed = *imu_msg;
  imu_transformed.linear_acceleration.x = transformed_accel.getX();
  imu_transformed.linear_acceleration.y = transformed_accel.getY();
  imu_transformed.linear_acceleration.z = transformed_accel.getZ();
  imu_transformed.angular_velocity.x = transformed_angular_vel.getX();
  imu_transformed.angular_velocity.y = transformed_angular_vel.getY();
  imu_transformed.angular_velocity.z = transformed_angular_vel.getZ();

  imu_pub_->publish(imu_transformed);

  if (enable_pid) {
    pid_current_velocity.linear.x = imu_transformed.linear_acceleration.x;
    pid_current_velocity.linear.y = imu_transformed.linear_acceleration.y;
    pid_current_velocity.linear.z = imu_transformed.linear_acceleration.z;
    pid_current_velocity.angular.x = imu_transformed.angular_velocity.x;
    pid_current_velocity.angular.y = imu_transformed.angular_velocity.y;
    pid_current_velocity.angular.z = imu_transformed.angular_velocity.z;
  }
}

void LidarTransform::lidar_callback(const livox_ros_driver2::msg::CustomMsg::SharedPtr lidar_msg)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  for (size_t i = 0; i < lidar_msg->point_num; ++i) {
    pcl::PointXYZ point;
    point.x = lidar_msg->points[i].x;
    point.y = lidar_msg->points[i].y;
    point.z = lidar_msg->points[i].z;
    pcl_cloud->points.push_back(point);
  }
  pcl_cloud->width = pcl_cloud->points.size();
  pcl_cloud->height = 1;

  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl_ros::transformPointCloud(*pcl_cloud, *transformed_cloud, frame_transform);

  for (size_t i = 0; i < transformed_cloud->points.size(); ++i) {
    lidar_msg->points[i].x = transformed_cloud->points[i].x;
    lidar_msg->points[i].y = transformed_cloud->points[i].y;
    lidar_msg->points[i].z = transformed_cloud->points[i].z;
  }

  lidar_pub_->publish(*lidar_msg);
}

void LidarTransform::cmd_vel_callback(const geometry_msgs::msg::TwistStamped::SharedPtr cmd_vel_msg)
{
  if (enable_pid) {
    pid_target_velocity = cmd_vel_msg->twist;

    geometry_msgs::msg::Twist error_velocity = geometry_msgs::msg::Twist();
    error_velocity.linear.x = pid_target_velocity.linear.x - pid_current_velocity.linear.x;
    error_velocity.linear.y = pid_target_velocity.linear.y - pid_current_velocity.linear.y;
    error_velocity.linear.z = pid_target_velocity.linear.z - pid_current_velocity.linear.z;
    error_velocity.angular.x = pid_target_velocity.angular.x - pid_current_velocity.angular.x;
    error_velocity.angular.y = pid_target_velocity.angular.y - pid_current_velocity.angular.y;
    error_velocity.angular.z = pid_target_velocity.angular.z - pid_current_velocity.angular.z;

    double dt = 0.1;
    pid_integral.linear.x += error_velocity.linear.x * dt;
    pid_integral.linear.y += error_velocity.linear.y * dt;
    pid_integral.linear.z += error_velocity.linear.z * dt;
    pid_integral.angular.x += error_velocity.angular.x * dt;
    pid_integral.angular.y += error_velocity.angular.y * dt;
    pid_integral.angular.z += error_velocity.angular.z * dt;

    geometry_msgs::msg::Twist pid_output = geometry_msgs::msg::Twist();
    pid_output.linear.x = pid_kp * error_velocity.linear.x + pid_ki * pid_integral.linear.x +
      pid_kd * (error_velocity.linear.x - pid_prev_error.linear.x) / dt;
    pid_output.linear.y = pid_kp * error_velocity.linear.y + pid_ki * pid_integral.linear.y +
      pid_kd * (error_velocity.linear.y - pid_prev_error.linear.y) / dt;
    pid_output.linear.z = pid_kp * error_velocity.linear.z + pid_ki * pid_integral.linear.z +
      pid_kd * (error_velocity.linear.z - pid_prev_error.linear.z) / dt;
    pid_output.angular.x = pid_kp * error_velocity.angular.x + pid_ki * pid_integral.angular.x +
      pid_kd * (error_velocity.angular.x - pid_prev_error.angular.x) / dt;
    pid_output.angular.y = pid_kp * error_velocity.angular.y + pid_ki * pid_integral.angular.y +
      pid_kd * (error_velocity.angular.y - pid_prev_error.angular.y) / dt;
    pid_output.angular.z = pid_kp * error_velocity.angular.z + pid_ki * pid_integral.angular.z +
      pid_kd * (error_velocity.angular.z - pid_prev_error.angular.z) / dt;

    geometry_msgs::msg::TwistStamped pid_output_msg = *cmd_vel_msg;
    pid_output_msg.twist = pid_output;
    cmd_vel_pub_->publish(pid_output_msg);

    pid_prev_error = error_velocity;
  }else{
    cmd_vel_pub_->publish(*cmd_vel_msg);
  }
}

} // namespace lidar_transform

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(lidar_transform::LidarTransform)