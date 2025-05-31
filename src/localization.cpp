// Copyright 2019 coderkarl. Subject to the BSD license.

#include "tools_localization/localization.hpp"
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
Localization::Localization() : Node("localization"), count_(0)
{
  sub_loc_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "/navsat", 10, std::bind(&Localization::GpsCallBack, this, _1));
  sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu", 10, std::bind(&Localization::ImuCallBack, this, _1));
  sub_slam_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/pose", 10, std::bind(&Localization::SlamCallBack, this, _1));
  sub_twist_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, std::bind(&Localization::TwistCallBack, this, _1));
  clock_sub_ = this->create_subscription<rosgraph_msgs::msg::Clock>(
      "/clock", 10,
      std::bind(&Localization::clockCallback, this, std::placeholders::_1)
  );
  tfB_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  publisher_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
  timer_ = this->create_wall_timer(
      100ms, std::bind(&Localization::timer_callback, this));
}

Localization::~Localization() {}

void Localization::timer_callback()
{
    // Calculate time difference in seconds
    // double dt = 0.1;
    
    // // Simple motion model (dead reckoning)
    theta_ += (twist_.angular.z * dt_ * k_theta_);
    x_ += twist_.linear.x * cos(theta_) * dt_ * k_speed_;
    y_ += twist_.linear.x * sin(theta_) * dt_ * k_speed_;
    
    // // Normalize angle
    // theta_ = normalize_angle(theta_);
    
    // // Create quaternion from yaw
    // tf2::Quaternion q;
    // q.setRPY(0.0, 0.0, 0.0);
    
    // // Create and publish transform if enabled
    // nanosec_+=100000;
    geometry_msgs::msg::TransformStamped transform;
    // transform.header.stamp = this->get_clock()->now();
    transform.header.stamp.sec = last_clock_time_.seconds();
    transform.header.stamp.nanosec = last_clock_time_.nanoseconds();
    transform.header.frame_id = "odom";
    transform.child_frame_id = "base_link";
    
    transform.transform.translation.x = x_;
    transform.transform.translation.y = y_;
    transform.transform.translation.z = 0;
    
    transform.transform.rotation.x = 0;
    transform.transform.rotation.y = 0;
    transform.transform.rotation.z = theta_;
    transform.transform.rotation.w = 1;
    
    tfB_->sendTransform(transform);
    
    // Create and publish odometry message
    auto odom = nav_msgs::msg::Odometry();
    odom.header.stamp.sec = sec_;
    odom.header.stamp.nanosec = nanosec_;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    // odom.pose = slam_pose_.pose;
    
    // Set position
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;
    
    odom.pose.pose.orientation.x = 0;
    odom.pose.pose.orientation.y = 0;
    odom.pose.pose.orientation.z = theta_;
    odom.pose.pose.orientation.w = 1;
    
    // // Set velocity
    // odom.twist.twist.linear.x = 0;
    // odom.twist.twist.linear.y = 0;
    // odom.twist.twist.angular.z = 0;
    
    // // Add covariance (simple diagonal for now)
    // odom.pose.covariance[0] = 0.1;  // x variance
    // odom.pose.covariance[7] = 0.1;  // y variance
    // odom.pose.covariance[35] = 0.1; // rotation variance
    
    publisher_odom_->publish(odom);
}

void Localization::TwistCallBack(const geometry_msgs::msg::Twist::SharedPtr msg_in)
{
  twist_ = *msg_in;
}

void Localization::clockCallback(const rosgraph_msgs::msg::Clock::SharedPtr msg)
{
    last_clock_time_ = msg->clock;
}

void Localization::ImuCallBack(const sensor_msgs::msg::Imu::SharedPtr msg_in){
  sensor_msgs::msg::Imu imu_pose = *msg_in;
  //  std::cout<<"ricevoimu "<<nanosec_<<"\n";
}

void Localization::SlamCallBack(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg_in)
{
  slam_pose_ = *msg_in;
  pose_received_ = true;
  std::cout<<"ricevo pose slam\n";
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Localization>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
