/*
 * slam_toolbox
 * Copyright Work Modifications (c) 2018, Simbe Robotics, Inc.
 * Copyright Work Modifications (c) 2019, Steve Macenski
 * Copyright Work Modifications (c) 2023, Daniel I. Meza
 *
 * THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THIS CREATIVE
 * COMMONS PUBLIC LICENSE ("CCPL" OR "LICENSE"). THE WORK IS PROTECTED BY
 * COPYRIGHT AND/OR OTHER APPLICABLE LAW. ANY USE OF THE WORK OTHER THAN AS
 * AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
 *
 * BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
 * BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
 * CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
 * CONDITIONS.
 *
 */

/* Author: Steve Macenski */

#include <memory>
#include "slam_toolbox/slam_toolbox_async_multirobot.hpp"

namespace slam_toolbox
{

/*****************************************************************************/
AsynchronousMultiRobotSlamToolbox::AsynchronousMultiRobotSlamToolbox(rclcpp::NodeOptions options)
: SlamToolbox(options), external_scan_topic_("/external_scan")
/*****************************************************************************/
{
  current_ns_ = this->get_namespace() + 1;

  // Create transformation buffer
  tf2_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
  rclcpp::sleep_for(std::chrono::seconds(5)); // Small delay to allow transform buffer to fill up

  // Create publisher for external laser scans
  external_scan_pub_ = this->create_publisher<slam_toolbox::msg::ExternalLaserScan>(external_scan_topic_, 10);

  // Create subscriber for external laser scans
  external_scan_sub_ = this->create_subscription<slam_toolbox::msg::ExternalLaserScan>(external_scan_topic_, 10, std::bind(&AsynchronousMultiRobotSlamToolbox::externalScanCallback, this, std::placeholders::_1));
}

/*****************************************************************************/
void AsynchronousMultiRobotSlamToolbox::laserCallback(
  sensor_msgs::msg::LaserScan::ConstSharedPtr scan)
/*****************************************************************************/
{
  // Store scan header
  scan_header = scan->header;

  // No odom info
  Pose2 pose;
  if (!pose_helper_->getOdomPose(pose, scan->header.stamp)) {
    RCLCPP_WARN(get_logger(), "Failed to compute odom pose");
    return;
  }

  // Ensure the laser can be used
  LaserRangeFinder* laser = getLaser(scan);
  if (!laser) {
    RCLCPP_WARN(get_logger(), "Failed to create laser device for"
      " %s; discarding scan", scan->header.frame_id.c_str());
    return;
  }

  // If valid, add scan
  if (shouldProcessScan(scan, pose)) {
    LocalizedRangeScan* range_scan = addScan(laser, scan, pose);

    // Publish as a external scan
    if (range_scan != nullptr) {
      local_map_ready_ = true;
      Matrix3 covariance;
      covariance.SetToIdentity();
      publishExternalScan(scan, laser->GetOffsetPose(), range_scan->GetOdometricPose(), covariance, scan->header.stamp);
    }
  }
}

/*****************************************************************************/
bool AsynchronousMultiRobotSlamToolbox::deserializePoseGraphCallback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<slam_toolbox::srv::DeserializePoseGraph::Request> req,
  std::shared_ptr<slam_toolbox::srv::DeserializePoseGraph::Response> resp)
/*****************************************************************************/
{
  if (req->match_type == procType::LOCALIZE_AT_POSE) {
    RCLCPP_WARN(get_logger(), "Requested a localization deserialization "
      "in non-localization mode.");
    return false;
  }

  return SlamToolbox::deserializePoseGraphCallback(request_header, req, resp);
}

/*****************************************************************************/
void AsynchronousMultiRobotSlamToolbox::publishExternalScan( 
  const sensor_msgs::msg::LaserScan::ConstSharedPtr &scan,
  const Pose2 &offset,
  const Pose2 &pose,
  const Matrix3 &cov,
  const rclcpp::Time &t)
/*****************************************************************************/
{
  slam_toolbox::msg::ExternalLaserScan external_scan_msg;

  // Construct scan portion of the message
  external_scan_msg.scan = *scan;
  external_scan_msg.scan.header.frame_id = scan->header.frame_id;  // scan frame

  // Construct scanner_offset portion of the message
  tf2::Quaternion q_offset(0., 0., 0., 1.0);
  q_offset.setRPY(0., 0., offset.GetHeading());
  tf2::Transform scanner_offset(q_offset, tf2::Vector3(offset.GetX(), offset.GetY(), 0.0));
  tf2::toMsg(scanner_offset, external_scan_msg.scanner_offset.transform);
  external_scan_msg.scanner_offset.header.stamp = t;
  external_scan_msg.scanner_offset.header.frame_id = base_frame_;    // base footprint frame.
  external_scan_msg.scanner_offset.child_frame_id = scan->header.frame_id;   // scan frame

  // Construct pose portion of the message
  tf2::Quaternion q(0., 0., 0., 1.0);
  q.setRPY(0., 0., pose.GetHeading());
  tf2::Transform transform(q, tf2::Vector3(pose.GetX(), pose.GetY(), 0.0));
  tf2::toMsg(transform, external_scan_msg.pose.pose.pose);   // relative to map frame

  external_scan_msg.pose.pose.covariance[0] = cov(0, 0) * position_covariance_scale_;  // x
  external_scan_msg.pose.pose.covariance[1] = cov(0, 1) * position_covariance_scale_;  // xy
  external_scan_msg.pose.pose.covariance[6] = cov(1, 0) * position_covariance_scale_;  // xy
  external_scan_msg.pose.pose.covariance[7] = cov(1, 1) * position_covariance_scale_;  // y
  external_scan_msg.pose.pose.covariance[35] = cov(2, 2) * yaw_covariance_scale_;      // yaw
  external_scan_msg.pose.header.stamp = t;
  external_scan_msg.pose.header.frame_id = map_frame_;

  external_scan_pub_->publish(external_scan_msg);
}

/*****************************************************************************/
void AsynchronousMultiRobotSlamToolbox::externalScanCallback(
  slam_toolbox::msg::ExternalLaserScan::ConstSharedPtr external_scan)
{
  // Get namespace from message, ignore those from oneself or if initial map has not been posted!
  std::string scan_ns = external_scan->scan.header.frame_id.substr(0, 
                          external_scan->scan.header.frame_id.find('/'));
  if (!local_map_ready_ || scan_ns == current_ns_) return;

  // Create standard laser scan message from external laser scan
  sensor_msgs::msg::LaserScan::ConstSharedPtr scan = 
    std::make_shared<sensor_msgs::msg::LaserScan>(external_scan->scan);

  // Create pose from external laser scan + local map frame transformation
  geometry_msgs::msg::TransformStamped transform_msg;
  Pose2 pose;
  try {
    transform_msg = tf2_buffer_->lookupTransform(map_frame_, external_scan->pose.header.frame_id, external_scan->scan.header.stamp, rclcpp::Duration::from_seconds(0.3));
  }
  catch(const tf2::TransformException &ex) {
    RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", map_frame_.c_str(), external_scan->pose.header.frame_id.c_str(), ex.what());
    return;
  }
  // Translation
  pose.SetX(external_scan->pose.pose.pose.position.x + transform_msg.transform.translation.x);
  pose.SetY(external_scan->pose.pose.pose.position.y + transform_msg.transform.translation.y);
  // Rotation
  tf2::Quaternion quat1, quat2;
  tf2::convert(external_scan->pose.pose.pose.orientation, quat1);
  tf2::convert(transform_msg.transform.rotation, quat2);
  pose.SetHeading(tf2::getYaw(quat1) + tf2::getYaw(quat2));

  // Ensure the laser can be used
  // TODO Change back if this doesn't work
  LaserRangeFinder* laser = getLaser(scan);
  if (!laser) {
    RCLCPP_WARN(get_logger(), "Failed to create laser device for "
      " %s; discarding scan", scan->header.frame_id.c_str());
    return;
  }

  // Since message is from different sensor, assume valid and process it
  LocalizedRangeScan* range_scan = addExternalScan(laser, scan, pose);
}

/*****************************************************************************/
LocalizedRangeScan* AsynchronousMultiRobotSlamToolbox::addExternalScan(
  LaserRangeFinder* laser,
  const sensor_msgs::msg::LaserScan::ConstSharedPtr &scan,
  Pose2 &odom_pose)
/*****************************************************************************/
{
  // Get our localized range scan
  LocalizedRangeScan* range_scan = getLocalizedRangeScan(
    laser, scan, odom_pose);

  // Add the localized range scan to the smapper
  boost::mutex::scoped_lock lock(smapper_mutex_);
  bool processed = false, update_reprocessing_transform = false;

  Matrix3 covariance;
  covariance.SetToIdentity();

  if (processor_type_ == PROCESS) {
    processed = smapper_->getMapper()->Process(range_scan, &covariance);
  } else if (processor_type_ == PROCESS_FIRST_NODE) {
    processed = smapper_->getMapper()->ProcessAtDock(range_scan, &covariance);
    processor_type_ = PROCESS;
    update_reprocessing_transform = true;
  } else if (processor_type_ == PROCESS_NEAR_REGION) {
    boost::mutex::scoped_lock l(pose_mutex_);
    if (!process_near_pose_) {
      RCLCPP_ERROR(get_logger(), "Process near region called without a "
        "valid region request. Ignoring scan.");
      return nullptr;
    }
    range_scan->SetOdometricPose(*process_near_pose_);
    range_scan->SetCorrectedPose(range_scan->GetOdometricPose());
    process_near_pose_.reset(nullptr);
    processed = smapper_->getMapper()->ProcessAgainstNodesNearBy(
      range_scan, false, &covariance);
    update_reprocessing_transform = true;
    processor_type_ = PROCESS;
  } else {
    RCLCPP_FATAL(get_logger(),
      "SlamToolbox: No valid processor type set! Exiting.");
    exit(-1);
  }

  // if successfully processed, create odom to map transformation
  // and add our scan to storage
  if (processed) {
    if (enable_interactive_mode_) {
      scan_holder_->addScan(*scan);
    }
  } else {
    delete range_scan;
    range_scan = nullptr;
  }

  return range_scan;
}

}  // namespace slam_toolbox
