/*
 * multirobot_slam_toolbox
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

/* Author: Steven Macenski */

#include "slam_toolbox/slam_toolbox_multirobot.hpp"

#include <memory>
namespace slam_toolbox
{

/*****************************************************************************/
MultiRobotSlamToolbox::MultiRobotSlamToolbox(rclcpp::NodeOptions options)
: SlamToolbox(options), external_scan_topic_("/scan_external")
/*****************************************************************************/
{
  current_ns_ = this->get_namespace() + 1;

  ssClear_ = this->create_service<slam_toolbox::srv::ClearQueue>("slam_toolbox/clear_queue",
    std::bind(&MultiRobotSlamToolbox::clearQueueCallback, this,
    std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  threads_.push_back(std::make_unique<boost::thread>(
    boost::bind(&MultiRobotSlamToolbox::run, this)));

  // Create publisher for external laser scans
  external_scan_pub_ = this->create_publisher<slam_toolbox::msg::ExternalLaserScan>(external_scan_topic_, 10);

  // Create subscriber for external laser scans
  external_scan_sub_ = this->create_subscription<slam_toolbox::msg::ExternalLaserScan>(external_scan_topic_, 10, std::bind(&MultiRobotSlamToolbox::externalScanCallback, this, std::placeholders::_1));
}

/*****************************************************************************/
void MultiRobotSlamToolbox::run()
/*****************************************************************************/
{
  rclcpp::Rate r(100);
  while (rclcpp::ok()) {
    if (!isPaused(PROCESSING)) {
      PosedScan scan_w_pose(nullptr, karto::Pose2()); // dummy, updated in critical section
      bool queue_empty = true;
      {
        boost::mutex::scoped_lock lock(q_mutex_);
        queue_empty = q_.empty();
        if (!queue_empty) {
          scan_w_pose = q_.front();
          q_.pop();

          if (q_.size() > 10) {
            RCLCPP_WARN(get_logger(), "Queue size has grown to: %i. "
              "Recommend stopping until message is gone if online mapping.",
              (int)q_.size());
          }
        }
      }
      if (!queue_empty) {
        LaserRangeFinder* laser = getLaser(scan_w_pose.scan);
        LocalizedRangeScan* range_scan = addScan(laser, scan_w_pose);
        if (range_scan != nullptr) {
          Matrix3 covariance;
          covariance.SetToIdentity();
          publishExternalScan(scan_w_pose.scan, laser->GetOffsetPose(), range_scan->GetOdometricPose(), covariance, scan_w_pose.scan->header.stamp);
        }

        continue;
      }
    }

    r.sleep();
  }
}

/*****************************************************************************/
void MultiRobotSlamToolbox::laserCallback(
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
    RCLCPP_WARN(get_logger(), "MultiRobotSlamToolbox: Failed to create laser device for %s; discarding scan", scan->header.frame_id.c_str());
    return;
  }

  // If sync and valid, add to queue
  if (shouldProcessScan(scan, pose)) {
    boost::mutex::scoped_lock lock(q_mutex_);
    q_.push(PosedScan(scan, pose));
  }

  // TODO Remove
  // Run SLAM algorithm to add map portion
  // LocalizedRangeScan* range_scan = addScan(laser, scan, pose);
  // if (range_scan != nullptr) {
  //   Matrix3 covariance;
  //   covariance.SetToIdentity();
  //   publishLocalizedScan(scan, laser->GetOffsetPose(), range_scan->GetOdometricPose(), covariance, scan->header.stamp);
  // }
}

/*****************************************************************************/
void MultiRobotSlamToolbox::externalScanCallback(
  slam_toolbox::msg::ExternalLaserScan::ConstSharedPtr external_scan)
{
  // Get namespace from message, ignore those from oneself!
  std::string scan_ns = external_scan->scan.header.frame_id.substr(0, 
                          external_scan->scan.header.frame_id.find('/'));
  if (scan_ns == current_ns_) return;

  // Create standard laser scan message from external laser scan
  // Add pose and heading angle
  sensor_msgs::msg::LaserScan::ConstSharedPtr scan = 
    std::make_shared<sensor_msgs::msg::LaserScan>(external_scan->scan);
  Pose2 pose;
  pose.SetX(external_scan->pose.pose.pose.position.x);
  pose.SetY(external_scan->pose.pose.pose.position.y);
  tf2::Quaternion quat_tf;
  tf2::convert(external_scan->pose.pose.pose.orientation, quat_tf);
  pose.SetHeading(tf2::getYaw(quat_tf));

  // Ensure the laser can be used
  LaserRangeFinder* laser = getExternalLaser(external_scan);
  if (!laser) {
    RCLCPP_WARN(get_logger(), "Failed to create device for received externalScanner"
      " %s; discarding scan", scan->header.frame_id.c_str());
    return;
  }

  // Since message is from different sensor, assume valid and process it
  LocalizedRangeScan* range_scan = addExternalScan(laser, scan, pose);
  if (range_scan != nullptr) {
    // Add pose and heading angle
    pose = range_scan->GetCorrectedPose();
    // tf2::Quaternion q(0., 0., 0., 1.0);
    // q.setRPY(0., 0., pose.GetHeading());
    // tf2::Transform transform(q, tf2::Vector3(pose.GetX(), pose.GetY(), 0.0));

    // Broadcast transform
    // geometry_msgs::msg::TransformStamped tf_msg;
    // tf2::toMsg(transform, tf_msg.transform);
    // tf_msg.header.frame_id = map_frame_;
    // tf_msg.header.stamp = localized_scan->pose.header.stamp;
    // tf_msg.child_frame_id = localized_scan->scanner_offset.header.frame_id;
    // tfB_->sendTransform(tf_msg);
  }
}

/*****************************************************************************/
LaserRangeFinder* MultiRobotSlamToolbox::getExternalLaser(
  const slam_toolbox::msg::ExternalLaserScan::ConstSharedPtr external_scan)
/*****************************************************************************/
{
  const std::string &frame = external_scan->scan.header.frame_id;

  if (lasers_.find(frame) == lasers_.end()) {
    try {
      lasers_[frame] = laser_assistant_->toLaserMetadata(external_scan->scan, external_scan->scanner_offset);
      dataset_->Add(lasers_[frame].getLaser(), true);
    } catch (tf2::TransformException & e) {
      RCLCPP_ERROR(get_logger(), "Failed to compute laser pose[%s], "
        "aborting initialization (%s)", frame.c_str(), e.what());
      return nullptr;
    }
  }

  return lasers_[frame].getLaser();
}

/*****************************************************************************/
LocalizedRangeScan* MultiRobotSlamToolbox::addExternalScan(
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

/*****************************************************************************/
void MultiRobotSlamToolbox::publishExternalScan( 
  const sensor_msgs::msg::LaserScan::ConstSharedPtr &scan,
  const Pose2 &offset,
  const Pose2 &pose,
  const Matrix3 &cov,
  const rclcpp::Time &t)
/*****************************************************************************/
{
  slam_toolbox::msg::ExternalLaserScan scan_msg;

  // Construct scan portion of the message
  scan_msg.scan = *scan;
  scan_msg.scan.header.frame_id = scan->header.frame_id;

  // Construct scanner_offset portion of the message
  tf2::Quaternion q_offset(0., 0., 0., 1.0);
  q_offset.setRPY(0., 0., offset.GetHeading());
  tf2::Transform scanner_offset(q_offset, tf2::Vector3(offset.GetX(), offset.GetY(), 0.0));
  tf2::toMsg(scanner_offset, scan_msg.scanner_offset.transform);
  scan_msg.scanner_offset.header.stamp = t;
  scan_msg.scanner_offset.header.frame_id = base_frame_;
  scan_msg.scanner_offset.child_frame_id = scan_msg.scan.header.frame_id;

  // Construct pose portion of the message
  tf2::Quaternion q(0., 0., 0., 1.0);
  q.setRPY(0., 0., pose.GetHeading());
  tf2::Transform transform(q, tf2::Vector3(pose.GetX(), pose.GetY(), 0.0));
  tf2::toMsg(transform, scan_msg.pose.pose.pose);

  scan_msg.pose.pose.covariance[0] = cov(0, 0) * position_covariance_scale_;  // x
  scan_msg.pose.pose.covariance[1] = cov(0, 1) * position_covariance_scale_;  // xy
  scan_msg.pose.pose.covariance[6] = cov(1, 0) * position_covariance_scale_;  // xy
  scan_msg.pose.pose.covariance[7] = cov(1, 1) * position_covariance_scale_;  // y
  scan_msg.pose.pose.covariance[35] = cov(2, 2) * yaw_covariance_scale_;      // yaw
  scan_msg.pose.header.stamp = t;
  scan_msg.pose.header.frame_id = map_frame_;

  // Set prefixed frame names
  // scan_msg.scan.header.frame_id = (*(scan->header.frame_id.cbegin()) == '/') ? 
  //   current_ns_ + scan->header.frame_id :
  //   current_ns_ + "/" + scan->header.frame_id;

  // scan_msg.pose.header.frame_id = (*(map_frame_.cbegin()) == '/') ? 
  //   current_ns_ + map_frame_ :
  //   current_ns_ + "/" + map_frame_;

  // scan_msg.scanner_offset.child_frame_id = scan_msg.scan.header.frame_id;

  // scan_msg.scanner_offset.header.frame_id = (*(base_frame_.cbegin()) == '/') ?
  //   current_ns_ + base_frame_ :
  //   current_ns_ + "/" + base_frame_;

  external_scan_pub_->publish(scan_msg);
}

/*****************************************************************************/
bool MultiRobotSlamToolbox::clearQueueCallback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<slam_toolbox::srv::ClearQueue::Request> req,
  std::shared_ptr<slam_toolbox::srv::ClearQueue::Response> resp)
/*****************************************************************************/
{
  RCLCPP_INFO(get_logger(), "MultiRobotSlamToolbox: "
    "Clearing all queued scans to add to map.");
  while (!q_.empty()) {
    q_.pop();
  }
  resp->status = true;
  return resp->status;
}

/*****************************************************************************/
bool MultiRobotSlamToolbox::deserializePoseGraphCallback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<slam_toolbox::srv::DeserializePoseGraph::Request> req,
  std::shared_ptr<slam_toolbox::srv::DeserializePoseGraph::Response> resp)
/*****************************************************************************/
{
  if (req->match_type == procType::LOCALIZE_AT_POSE) {
    RCLCPP_ERROR(get_logger(), "Requested a localization deserialization "
      "in non-localization mode.");
    return false;
  }

  return SlamToolbox::deserializePoseGraphCallback(request_header, req, resp);
}

}  // namespace slam_toolbox
