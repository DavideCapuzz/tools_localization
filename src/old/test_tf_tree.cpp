// Copyright 2019 coderkarl. Subject to the BSD license.

#include "tools_localization/test_tf_tree.hpp"
// #include "nav_sim/AvoidObsCommon.h"
#include <geometry_msgs/msg/point_stamped.h>
#include <math.h>
#include <boost/math/special_functions/round.hpp>
#include <algorithm>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

/**********************************************************************
 * Obstacle Avoidance using a nav_msgs/OccupacyGrid and A* path planning
 *
 * Subscribe to /scan and Publish OccupacyGrid /costmap, Publish Path /path
 *
 **********************************************************************/

using std::placeholders::_1;

// Constructor
TFTest::TFTest() : Node("TFTest")
{
  double tmp_val = 30.;
  tmp_val = this->declare_parameter("tf_buffer_duration", tmp_val);
  tf_ = std::make_unique<tf2_ros::Buffer>(this->get_clock(),
      tf2::durationFromSec(tmp_val));
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    get_node_base_interface(),
    get_node_timers_interface());
  tf_->setCreateTimerInterface(timer_interface);
  tfL_ = std::make_unique<tf2_ros::TransformListener>(*tf_);
auto transform_timeout_ = rclcpp::Duration::from_seconds(1.0);;
// scan_filter_sub_ =
//     std::make_unique<message_filters::Subscriber<sensor_msgs::msg::LaserScan>>(
//     shared_from_this().get(), "/scan", rmw_qos_profile_sensor_data);
//   scan_filter_ =
//     std::make_unique<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>(
//     *scan_filter_sub_, *tf_, "odom", 1.0, shared_from_this(),
//     tf2::durationFromSec(transform_timeout_.seconds()));

//   scan_filter_->registerCallback(
//     std::bind(&TFTest::laserCallback, this, std::placeholders::_1));
}

void TFTest::init()
{
  scan_filter_sub_ = std::make_unique<message_filters::Subscriber<sensor_msgs::msg::LaserScan>>(
    shared_from_this().get(), "/scan", rmw_qos_profile_sensor_data);
  
  scan_filter_ = std::make_unique<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>(
    *scan_filter_sub_, *tf_, "odom", 20, shared_from_this(),
    tf2::durationFromSec(1.0));

  scan_filter_->registerCallback(
    std::bind(&TFTest::laserCallback, this, std::placeholders::_1));
}

void TFTest::laserCallback(
  sensor_msgs::msg::LaserScan::ConstSharedPtr scan)
/*****************************************************************************/
{
  // // store scan header
  auto scan_header = scan->header;
  // // no odom info
  // Pose2 pose;
  // if (!pose_helper_->getOdomPose(pose, scan->header.stamp)) {
  //   RCLCPP_WARN(get_logger(), "Failed to compute odom pose");
  //   return;
  // }

  // // ensure the laser can be used
  // LaserRangeFinder * laser = getLaser(scan);

  // if (!laser) {
  //   RCLCPP_WARN(get_logger(), "Failed to create laser device for"
  //     " %s; discarding scan", scan->header.frame_id.c_str());
  //   return;
  // }

  // addScan(laser, scan, pose);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TFTest>();
  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
