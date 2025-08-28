// Copyright 2019 coderkarl. Subject to the BSD license.

#include "tools_localization/localization_node.hpp"
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
LocalizationNode::LocalizationNode() : Node("LocalizationNode"), count_(0)
{
  sub_loc_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "/navsat", 10, std::bind(&LocalizationNode::GpsCallBack, this, _1));
  sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu", 10, std::bind(&LocalizationNode::ImuCallBack, this, _1));
  sub_slam_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/pose", 10, std::bind(&LocalizationNode::SlamCallBack, this, _1));
  sub_twist_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, std::bind(&LocalizationNode::TwistCallBack, this, _1));
  clock_sub_ = this->create_subscription<rosgraph_msgs::msg::Clock>(
      "/clock", 10,
      std::bind(&LocalizationNode::clockCallback, this, std::placeholders::_1)
  );
  tfB_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  publisher_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
  timer_ = this->create_wall_timer(
      100ms, std::bind(&LocalizationNode::timer_callback, this));


    std::string config_file_path = "/home/davide/ros_ws/wheele/src/tools_localization/config/config.json";

    // make a dummy state
    StateVec initial_state;
    CovMat initial_covariance;

    initial_covariance.setZero();
    initial_covariance.diagonal() <<
    0.5, 0.5, 0.5,   // positions
    0.1, 0.1, 0.1,   // velocities
    0.25, 0.25, 0.25, // attitude angles
    1e-4, 1e-4, 1e-4,   // accelerometer biases
    1e-4, 1e-4, 1e-4;   // gyro biases

    initial_state = 1e-3 * Eigen::Matrix<double, N, 1>::Ones();

    estimator = std::make_unique<UKF>(config_file_path);
    estimator->initialize(initial_state, initial_covariance);
    estimator->start_filter();
}

LocalizationNode::~LocalizationNode() {}

void LocalizationNode::timer_callback()
{

    // auto [transform, odom] = filter_.update(twist_, last_clock_time_);
    // tfB_->sendTransform(transform);
    // publisher_odom_->publish(odom);
}

void LocalizationNode::TwistCallBack(const geometry_msgs::msg::Twist::SharedPtr msg_in)
{
  twist_ = *msg_in;
}

void LocalizationNode::clockCallback(const rosgraph_msgs::msg::Clock::SharedPtr msg)
{
    last_clock_time_ = msg->clock;
}

void LocalizationNode::ImuCallBack(const sensor_msgs::msg::Imu::SharedPtr msg_in){
  imu_pose_ = *msg_in;
}

void LocalizationNode::GpsCallBack(const sensor_msgs::msg::NavSatFix::SharedPtr msg_in){
  gps_ = *msg_in;
}

void LocalizationNode::SlamCallBack(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg_in)
{
  slam_pose_ = *msg_in;
  pose_received_ = true;
  std::cout<<"ricevo pose slam\n";
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LocalizationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
