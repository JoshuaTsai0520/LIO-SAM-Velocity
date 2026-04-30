#ifndef DISTORTION_FUNCTION_HPP_
#define DISTORTION_FUNCTION_HPP_

#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>
#include <sophus/se3.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <tf2_ros/buffer.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <deque>
#include <memory>
#include <string>

class DistortionFunctionBase
{
public:
  virtual ~DistortionFunctionBase() = default;

  virtual void processTwistMessage(
    const geometry_msgs::msg::TwistStamped::ConstSharedPtr twist_msg) = 0;
  virtual void processIMUMessage(
    const std::string & base_frame, const sensor_msgs::msg::Imu imu_msg, bool use_velocity) = 0;
  virtual void setPointCloudTransform(
    const std::string & base_frame, const std::string & lidar_frame) = 0;
  virtual void initialize() = 0;
  virtual void undistortPointCloud(bool use_imu, sensor_msgs::msg::PointCloud2 & pointcloud, pcl::PointCloud<pcl::PointXYZI>::Ptr & FirstPoint) = 0;
};

template <class T>
class DistortionFunction : public DistortionFunctionBase
{
public:
  bool pointcloud_transform_needed_{false};
  bool pointcloud_transform_exists_{false};
  bool imu_transform_exists_{false};
  geometry_msgs::msg::TransformStamped imu_to_base_link_transform_;
  rclcpp::Node * node_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::deque<geometry_msgs::msg::TwistStamped> twist_queue_;
  std::deque<geometry_msgs::msg::Vector3Stamped> angular_velocity_queue_;

  explicit DistortionFunction(rclcpp::Node * node)
  : node_(node), tf_buffer_(node_->get_clock()), tf_listener_(tf_buffer_)
  {
  }
  void processTwistMessage(
    const geometry_msgs::msg::TwistStamped::ConstSharedPtr twist_msg) override;

  void processIMUMessage(
    const std::string & base_frame, const sensor_msgs::msg::Imu imu_msg, bool use_velocity) override;
  void getIMUTransformation(
    const std::string & base_frame, const std::string & imu_frame,
    geometry_msgs::msg::TransformStamped::SharedPtr geometry_imu_to_base_link_ptr);
  void enqueueIMU(
    const sensor_msgs::msg::Imu imu_msg,
    geometry_msgs::msg::TransformStamped::SharedPtr geometry_imu_to_base_link_ptr, bool use_velocity);

  bool isInputValid(bool use_imu, sensor_msgs::msg::PointCloud2 & pointcloud);
  void getTwistAndIMUIterator(
    bool use_imu, double first_point_time_stamp_sec,
    std::deque<geometry_msgs::msg::TwistStamped>::iterator & it_twist,
    std::deque<geometry_msgs::msg::Vector3Stamped>::iterator & it_imu);
  void undistortPointCloud(bool use_imu, sensor_msgs::msg::PointCloud2 & pointcloud, pcl::PointCloud<pcl::PointXYZI>::Ptr & FirstPoint) override;
  template <typename TimeIterator>
  void undistortPointCloudWithTimeIterator(
    bool use_imu, sensor_msgs::msg::PointCloud2 & pointcloud,
    pcl::PointCloud<pcl::PointXYZI>::Ptr & FirstPoint, TimeIterator & it_time_stamp,
    double time_base_sec);
  void warnIfTimestampIsTooLate(bool is_twist_time_stamp_too_late, bool is_imu_time_stamp_too_late);
  void undistortPoint(
    sensor_msgs::PointCloud2Iterator<float> & it_x, sensor_msgs::PointCloud2Iterator<float> & it_y,
    sensor_msgs::PointCloud2Iterator<float> & it_z,
    std::deque<geometry_msgs::msg::TwistStamped>::iterator & it_twist,
    std::deque<geometry_msgs::msg::Vector3Stamped>::iterator & it_imu, float const & time_offset,
    const bool & is_twist_valid, const bool & is_imu_valid)
  {
    static_cast<T *>(this)->undistortPointImplementation(
      it_x, it_y, it_z, it_twist, it_imu, time_offset, is_twist_valid, is_imu_valid);
  };
};

class DistortionFunction3D : public DistortionFunction<DistortionFunction3D>
{
private:
  // defined outside of for loop for performance reasons.
  Eigen::Vector4f point_eigen_;
  Eigen::Vector4f undistorted_point_eigen_;
  Eigen::Matrix4f transformation_matrix_;
  Eigen::Matrix4f prev_transformation_matrix_;

  // TF
  Eigen::Matrix4f eigen_lidar_to_base_link_;
  Eigen::Matrix4f eigen_base_link_to_lidar_;

public:
  explicit DistortionFunction3D(rclcpp::Node * node) : DistortionFunction(node) {}
  void initialize() override;
  void undistortPointImplementation(
    sensor_msgs::PointCloud2Iterator<float> & it_x, sensor_msgs::PointCloud2Iterator<float> & it_y,
    sensor_msgs::PointCloud2Iterator<float> & it_z,
    std::deque<geometry_msgs::msg::TwistStamped>::iterator & it_twist,
    std::deque<geometry_msgs::msg::Vector3Stamped>::iterator & it_imu, const float & time_offset,
    const bool & is_twist_valid, const bool & is_imu_valid);
  void setPointCloudTransform(
    const std::string & base_frame, const std::string & lidar_frame) override;
};

#endif  // DISTORTION_FUNCTION_HPP_
