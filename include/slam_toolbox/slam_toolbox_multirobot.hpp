/*
 * multirobot_slam_toolbox
 * Copyright Work Modifications (c) 2023, Achala Athukorala
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

#ifndef SLAM_TOOLBOX__SLAM_TOOLBOX_MULTIROBOT_HPP_
#define SLAM_TOOLBOX__SLAM_TOOLBOX_MULTIROBOT_HPP_

#include <queue>
#include <memory>
#include "slam_toolbox/slam_toolbox_common.hpp"
#include "slam_toolbox/toolbox_msgs.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace slam_toolbox
{

class MultiRobotSlamToolbox : public SlamToolbox
{
  public:
    explicit MultiRobotSlamToolbox(rclcpp::NodeOptions options);
    ~MultiRobotSlamToolbox() {}
    void run();


  protected:
    void laserCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr scan) override;

    void externalScanCallback(slam_toolbox::msg::ExternalLaserScan::ConstSharedPtr external_scan);

    LaserRangeFinder* getExternalLaser(
      const slam_toolbox::msg::ExternalLaserScan::ConstSharedPtr external_scan);

    LocalizedRangeScan* addExternalScan(
      LaserRangeFinder* laser,
      const sensor_msgs::msg::LaserScan::ConstSharedPtr &scan,
      Pose2 &odom_pose);

    void publishExternalScan(
      const sensor_msgs::msg::LaserScan::ConstSharedPtr &scan,
      const Pose2 &offset,
      const Pose2 &pose, const Matrix3 &cov,
      const rclcpp::Time &t);

    bool clearQueueCallback(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<slam_toolbox::srv::ClearQueue::Request> req,
      std::shared_ptr<slam_toolbox::srv::ClearQueue::Response> resp);

    bool deserializePoseGraphCallback(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<slam_toolbox::srv::DeserializePoseGraph::Request> req,
      std::shared_ptr<slam_toolbox::srv::DeserializePoseGraph::Response> resp) override;


    std::queue<PosedScan> q_;
    std::shared_ptr<rclcpp::Service<slam_toolbox::srv::ClearQueue>> ssClear_;
    boost::mutex q_mutex_;
    std::string external_scan_topic_;
    std::string current_ns_;
    bool local_map_ready_ = false;
    std::unique_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_{nullptr};
    std::shared_ptr<rclcpp::Publisher<slam_toolbox::msg::ExternalLaserScan>> external_scan_pub_;
    rclcpp::Subscription<slam_toolbox::msg::ExternalLaserScan>::SharedPtr external_scan_sub_;
};

}  // namespace slam_toolbox

#endif   // SLAM_TOOLBOX__SLAM_TOOLBOX_MULTIROBOT_HPP_
