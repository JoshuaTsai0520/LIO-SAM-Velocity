#include "lio_sam/distortion_function.hpp"

template <class T>
void DistortionFunction<T>::processTwistMessage(
  const geometry_msgs::msg::TwistStamped::ConstSharedPtr twist_msg)
{
  geometry_msgs::msg::TwistStamped msg;
  msg.header = twist_msg->header;
  msg.twist = twist_msg->twist;
  twist_queue_.push_back(msg);

  while (!twist_queue_.empty()) {
    // for replay rosbag
    if (rclcpp::Time(twist_queue_.front().header.stamp) > rclcpp::Time(twist_msg->header.stamp)) {
      twist_queue_.pop_front();
    } else if (  // NOLINT
      rclcpp::Time(twist_queue_.front().header.stamp) <
      rclcpp::Time(twist_msg->header.stamp) - rclcpp::Duration::from_seconds(1.0)) {
      twist_queue_.pop_front();
    } else {
      break;
    }
  }
}

template <class T>
void DistortionFunction<T>::processIMUMessage(
  const std::string & base_frame, const sensor_msgs::msg::Imu imu_msg, bool use_velocity)
{
  geometry_msgs::msg::TransformStamped::SharedPtr geometry_imu_to_base_link_ptr =
    std::make_shared<geometry_msgs::msg::TransformStamped>();
  getIMUTransformation(base_frame, imu_msg.header.frame_id, geometry_imu_to_base_link_ptr);
  enqueueIMU(imu_msg, geometry_imu_to_base_link_ptr, use_velocity);
}

template <class T>
void DistortionFunction<T>::getIMUTransformation(
  const std::string & base_frame, const std::string & imu_frame,
  geometry_msgs::msg::TransformStamped::SharedPtr geometry_imu_to_base_link_ptr)
{
  if (imu_transform_exists_) {
    *geometry_imu_to_base_link_ptr = imu_to_base_link_transform_;
    return;
  }

  tf2::Transform tf2_imu_to_base_link;
  if (base_frame == imu_frame) {
    tf2_imu_to_base_link.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
    tf2_imu_to_base_link.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
    imu_transform_exists_ = true;
  } else {
    try {
      const auto transform_msg =
        tf_buffer_.lookupTransform(base_frame, imu_frame, tf2::TimePointZero);
      tf2::convert(transform_msg.transform, tf2_imu_to_base_link);
      imu_transform_exists_ = true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(node_->get_logger(), "%s", ex.what());
      RCLCPP_ERROR(
        node_->get_logger(), "Please publish TF %s to %s", base_frame.c_str(), imu_frame.c_str());

      tf2_imu_to_base_link.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
      tf2_imu_to_base_link.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
    }
  }

  imu_to_base_link_transform_.header.frame_id = base_frame;
  imu_to_base_link_transform_.child_frame_id = imu_frame;
  imu_to_base_link_transform_.transform.rotation = tf2::toMsg(tf2_imu_to_base_link.getRotation());
  *geometry_imu_to_base_link_ptr = imu_to_base_link_transform_;
}

template <class T>
void DistortionFunction<T>::enqueueIMU(
  const sensor_msgs::msg::Imu imu_msg,
  geometry_msgs::msg::TransformStamped::SharedPtr geometry_imu_to_base_link_ptr, bool use_velocity)
{
  geometry_msgs::msg::Vector3Stamped angular_velocity;
  angular_velocity.vector = imu_msg.angular_velocity;

  geometry_msgs::msg::Vector3Stamped transformed_angular_velocity;
  tf2::doTransform(angular_velocity, transformed_angular_velocity, *geometry_imu_to_base_link_ptr);
  transformed_angular_velocity.header = imu_msg.header;
  angular_velocity_queue_.push_back(transformed_angular_velocity);

  while (!angular_velocity_queue_.empty()) {
    // for replay rosbag
    if (
      rclcpp::Time(angular_velocity_queue_.front().header.stamp) >
      rclcpp::Time(imu_msg.header.stamp)) {
      angular_velocity_queue_.pop_front();
    } else if (  // NOLINT
      rclcpp::Time(angular_velocity_queue_.front().header.stamp) <
      rclcpp::Time(imu_msg.header.stamp) - rclcpp::Duration::from_seconds(1.0)) {
      angular_velocity_queue_.pop_front();
    } else {
      break;
    }
  }

  if (!use_velocity){
    geometry_msgs::msg::TwistStamped msg;
    msg.header = imu_msg.header;
    msg.twist.linear.x = 0;
    msg.twist.linear.y = 0.0;
    msg.twist.linear.z = 0.0;
    msg.twist.angular.x = 0.0;
    msg.twist.angular.y = 0.0;
    msg.twist.angular.z = 0.0;
    twist_queue_.push_back(msg);

    while (!twist_queue_.empty()) {
      // for replay rosbag
      if (rclcpp::Time(twist_queue_.front().header.stamp) > rclcpp::Time(imu_msg.header.stamp)) {
        twist_queue_.pop_front();
      } else if (  // NOLINT
        rclcpp::Time(twist_queue_.front().header.stamp) <
        rclcpp::Time(imu_msg.header.stamp) - rclcpp::Duration::from_seconds(1.0)) {
        twist_queue_.pop_front();
      } else {
        break;
      }
    }
  }
}

template <class T>
void DistortionFunction<T>::getTwistAndIMUIterator(
  bool use_imu, double first_point_time_stamp_sec,
  std::deque<geometry_msgs::msg::TwistStamped>::iterator & it_twist,
  std::deque<geometry_msgs::msg::Vector3Stamped>::iterator & it_imu)
{
  if (!twist_queue_.empty()) {
    it_twist = std::lower_bound(
      twist_queue_.begin(), twist_queue_.end(), first_point_time_stamp_sec,
      [](const geometry_msgs::msg::TwistStamped & x, const double t) {
        return rclcpp::Time(x.header.stamp).seconds() < t;
      });
    if (it_twist == twist_queue_.end() && twist_queue_.size() > 1) {
      --it_twist;
    }
  } else {
    it_twist = twist_queue_.end();
  }

  if (use_imu && !angular_velocity_queue_.empty()) {
    it_imu = std::lower_bound(
      angular_velocity_queue_.begin(), angular_velocity_queue_.end(), first_point_time_stamp_sec,
      [](const geometry_msgs::msg::Vector3Stamped & x, const double t) {
        return rclcpp::Time(x.header.stamp).seconds() < t;
      });
    if (it_imu == angular_velocity_queue_.end() && angular_velocity_queue_.size() > 1) {
      --it_imu;
    }
  } else {
    it_imu = angular_velocity_queue_.end();
  }
}

template <class T>
bool DistortionFunction<T>::isInputValid(bool use_imu, sensor_msgs::msg::PointCloud2 & pointcloud)
{
  if (pointcloud.data.empty() || twist_queue_.empty() ||
    (use_imu && angular_velocity_queue_.empty()))
  {
    RCLCPP_WARN_STREAM_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 10000 /* ms */,
      "input pointcloud or required motion queue is empty.");
    return false;
  }
  // std::cout << twist_queue_.size() << ", " << angular_velocity_queue_.size() << std::endl;

  auto time_stamp_field_it = std::find_if(
    std::cbegin(pointcloud.fields), std::cend(pointcloud.fields),
    [](const sensor_msgs::msg::PointField & field) { return field.name == "time_stamp"; });
  auto time_field_it = std::find_if(
    std::cbegin(pointcloud.fields), std::cend(pointcloud.fields),
    [](const sensor_msgs::msg::PointField & field) { return field.name == "time"; });
  if (time_stamp_field_it == pointcloud.fields.cend() &&
    time_field_it == pointcloud.fields.cend())
  {
    RCLCPP_WARN_STREAM_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 10000 /* ms */,
      "Required field time_stamp or time doesn't exist in the point cloud.");
    return false;
  }
  return true;
}

template <class T>
void DistortionFunction<T>::undistortPointCloud(
  bool use_imu, sensor_msgs::msg::PointCloud2 & pointcloud, pcl::PointCloud<pcl::PointXYZI>::Ptr & FirstPoint)
{
  if (!isInputValid(use_imu, pointcloud)) return;

  auto time_stamp_field_it = std::find_if(
    std::cbegin(pointcloud.fields), std::cend(pointcloud.fields),
    [](const sensor_msgs::msg::PointField & field) { return field.name == "time_stamp"; });
  if (time_stamp_field_it != pointcloud.fields.cend()) {
    sensor_msgs::PointCloud2ConstIterator<double> it_time_stamp(pointcloud, "time_stamp");
    undistortPointCloudWithTimeIterator(use_imu, pointcloud, FirstPoint, it_time_stamp, 0.0);
    return;
  }

  sensor_msgs::PointCloud2ConstIterator<float> it_time(pointcloud, "time");
  undistortPointCloudWithTimeIterator(
    use_imu, pointcloud, FirstPoint, it_time, rclcpp::Time(pointcloud.header.stamp).seconds());
}

template <class T>
template <typename TimeIterator>
void DistortionFunction<T>::undistortPointCloudWithTimeIterator(
  bool use_imu, sensor_msgs::msg::PointCloud2 & pointcloud,
  pcl::PointCloud<pcl::PointXYZI>::Ptr & FirstPoint, TimeIterator & it_time_stamp,
  double time_base_sec)
{
  sensor_msgs::PointCloud2Iterator<float> it_x(pointcloud, "x");
  sensor_msgs::PointCloud2Iterator<float> it_y(pointcloud, "y");
  sensor_msgs::PointCloud2Iterator<float> it_z(pointcloud, "z");

  double prev_time_stamp_sec{time_base_sec + *it_time_stamp};
  const double first_point_time_stamp_sec{prev_time_stamp_sec};

  std::deque<geometry_msgs::msg::TwistStamped>::iterator it_twist;
  std::deque<geometry_msgs::msg::Vector3Stamped>::iterator it_imu;
  getTwistAndIMUIterator(use_imu, first_point_time_stamp_sec, it_twist, it_imu);

  // For performance, do not instantiate `rclcpp::Time` inside of the for-loop
  const double time_tolerance = 0.1;

  double twist_stamp = rclcpp::Time(it_twist->header.stamp).seconds();
  double imu_stamp{0.0};
  if (use_imu && !angular_velocity_queue_.empty()) {
    imu_stamp = rclcpp::Time(it_imu->header.stamp).seconds();
  }

  // If there is a point in a pointcloud that cannot be associated, record it to issue a warning
  bool is_twist_time_stamp_too_late = false;
  bool is_imu_time_stamp_too_late = false;
  bool is_twist_valid = true;
  bool is_imu_valid = true;
  bool is_first_point = true;

  for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z, ++it_time_stamp) {
    const double point_time_stamp_sec = time_base_sec + *it_time_stamp;
    float valid_range = float(std::sqrt(*it_x * *it_x + *it_y * *it_y + *it_z * *it_z));
    if (valid_range < 3.0){
      continue;
    }
    is_twist_valid = true;
    is_imu_valid = true;

    // Get closest twist information
    if (!twist_queue_.empty()) {
      while (it_twist != twist_queue_.end() && std::distance(it_twist, std::end(twist_queue_)) > 1 && point_time_stamp_sec > twist_stamp) {
        ++it_twist;
        twist_stamp = rclcpp::Time(it_twist->header.stamp).seconds();
      }
    }
    if (std::abs(point_time_stamp_sec - twist_stamp) > time_tolerance) {
      is_twist_time_stamp_too_late = true;
      is_twist_valid = false;
    }

    if (use_imu && !angular_velocity_queue_.empty()) {
      while (it_imu != angular_velocity_queue_.end() && std::distance(it_imu, std::end(angular_velocity_queue_)) > 1 && point_time_stamp_sec > imu_stamp) {
        ++it_imu;
        imu_stamp = rclcpp::Time(it_imu->header.stamp).seconds();
      }
      if (std::abs(point_time_stamp_sec - imu_stamp) > time_tolerance) {
        is_imu_time_stamp_too_late = true;
        is_imu_valid = false;
      }
    } else {
      is_imu_valid = false;
    }

    float time_offset = static_cast<float>(point_time_stamp_sec - prev_time_stamp_sec);

    // Undistort a single point based on the strategy
    undistortPoint(it_x, it_y, it_z, it_twist, it_imu, time_offset, is_twist_valid, is_imu_valid);

    prev_time_stamp_sec = point_time_stamp_sec;

    if (is_first_point) {
      FirstPoint->clear();
      pcl::PointXYZI thisPoint;
      thisPoint.x = *it_x;
      thisPoint.y = *it_y;
      thisPoint.z = *it_z;
      FirstPoint->push_back(thisPoint);
      is_first_point = false;
    }
  }

  warnIfTimestampIsTooLate(is_twist_time_stamp_too_late, is_imu_time_stamp_too_late);
}

template <class T>
void DistortionFunction<T>::warnIfTimestampIsTooLate(
  bool is_twist_time_stamp_too_late, bool is_imu_time_stamp_too_late)
{
  if (is_twist_time_stamp_too_late) {
    RCLCPP_WARN_STREAM_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 10000 /* ms */,
      "Twist time_stamp is too late. Could not interpolate.");
  }

  if (is_imu_time_stamp_too_late) {
    RCLCPP_WARN_STREAM_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 10000 /* ms */,
      "IMU time_stamp is too late. Could not interpolate.");
  }
}

void DistortionFunction3D::initialize()
{
  prev_transformation_matrix_ = Eigen::Matrix4f::Identity();
}

void DistortionFunction3D::setPointCloudTransform(
  const std::string & base_frame, const std::string & lidar_frame)
{
  if (pointcloud_transform_exists_) {
    return;
  }

  if (base_frame == lidar_frame) {
    eigen_lidar_to_base_link_ = Eigen::Matrix4f::Identity();
    eigen_base_link_to_lidar_ = Eigen::Matrix4f::Identity();
    pointcloud_transform_exists_ = true;
    return;
  }

  try {
    const auto transform_msg =
      tf_buffer_.lookupTransform(base_frame, lidar_frame, tf2::TimePointZero);
    eigen_lidar_to_base_link_ =
      tf2::transformToEigen(transform_msg.transform).matrix().cast<float>();
    eigen_base_link_to_lidar_ = eigen_lidar_to_base_link_.inverse();
    pointcloud_transform_exists_ = true;
    pointcloud_transform_needed_ = true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(node_->get_logger(), "%s", ex.what());
    RCLCPP_ERROR(
      node_->get_logger(), "Please publish TF %s to %s", base_frame.c_str(), lidar_frame.c_str());
    eigen_lidar_to_base_link_ = Eigen::Matrix4f::Identity();
    eigen_base_link_to_lidar_ = Eigen::Matrix4f::Identity();
  }
}


inline void DistortionFunction3D::undistortPointImplementation(
  sensor_msgs::PointCloud2Iterator<float> & it_x, sensor_msgs::PointCloud2Iterator<float> & it_y,
  sensor_msgs::PointCloud2Iterator<float> & it_z,
  std::deque<geometry_msgs::msg::TwistStamped>::iterator & it_twist,
  std::deque<geometry_msgs::msg::Vector3Stamped>::iterator & it_imu, const float & time_offset,
  const bool & is_twist_valid, const bool & is_imu_valid)
{
  // Initialize linear velocity and angular velocity
  float v_x_{0.0f}, v_y_{0.0f}, v_z_{0.0f}, w_x_{0.0f}, w_y_{0.0f}, w_z_{0.0f};
  if (is_twist_valid && it_twist != twist_queue_.end()) {
    v_x_ = static_cast<float>(it_twist->twist.linear.x);
    v_y_ = static_cast<float>(it_twist->twist.linear.y);
    v_z_ = static_cast<float>(it_twist->twist.linear.z);
    w_x_ = static_cast<float>(it_twist->twist.angular.x);
    w_y_ = static_cast<float>(it_twist->twist.angular.y);
    w_z_ = static_cast<float>(it_twist->twist.angular.z);
  }
  if (is_imu_valid && it_imu != angular_velocity_queue_.end()) {
    w_x_ = static_cast<float>(it_imu->vector.x);
    w_y_ = static_cast<float>(it_imu->vector.y);
    w_z_ = static_cast<float>(it_imu->vector.z);
  }

  // Undistort point
  point_eigen_ << *it_x, *it_y, *it_z, 1.0;
  if (pointcloud_transform_needed_) {
    point_eigen_ = eigen_lidar_to_base_link_ * point_eigen_;
  }

  Sophus::SE3f::Tangent twist(v_x_, v_y_, v_z_, w_x_, w_y_, w_z_);
  twist = twist * time_offset;
  transformation_matrix_ = Sophus::SE3f::exp(twist).matrix();
  transformation_matrix_ = transformation_matrix_ * prev_transformation_matrix_;
  undistorted_point_eigen_ = transformation_matrix_ * point_eigen_;

  if (pointcloud_transform_needed_) {
    undistorted_point_eigen_ = eigen_base_link_to_lidar_ * undistorted_point_eigen_;
  }
  *it_x = undistorted_point_eigen_[0];
  *it_y = undistorted_point_eigen_[1];
  *it_z = undistorted_point_eigen_[2];

  prev_transformation_matrix_ = transformation_matrix_;
}

template class DistortionFunction<DistortionFunction3D>;
