// Copyright 2019 coderkarl. Subject to the BSD license.

#include "tools_localization/pose.hpp"
//#include "nav_sim/AvoidObsCommon.h"
#include <geometry_msgs/msg/point_stamped.h>
#include <math.h>
#include <boost/math/special_functions/round.hpp>
#include <algorithm>

/**********************************************************************
* Obstacle Avoidance using a nav_msgs/OccupacyGrid and A* path planning
* 
* Subscribe to /scan and Publish OccupacyGrid /costmap, Publish Path /path
* 
**********************************************************************/

using std::placeholders::_1;

//Constructor
Pose::Pose() :
    Node("pose_calculator"), count_(0)
{
    publisher_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&Pose::timer_callback, this));
}

Pose::~Pose(){}

void Pose::timer_callback()
{
    auto message = nav_msgs::msg::Odometry();
    message.child_frame_id = "map";
    message.header.frame_id="odom";
    //message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: ");
    publisher_odom_->publish(message);
}
      
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Pose>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
