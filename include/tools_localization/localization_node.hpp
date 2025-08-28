#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <sys/time.h>
#include <fstream>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <ctime>
#include <iomanip>
#include <chrono>
#include <thread>

#include <rosgraph_msgs/msg/clock.hpp>
#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include "tools_localization/filters/UKF.hpp"
#include "geodetic_utils/geodetic_conv.hpp"
using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class LocalizationNode : public rclcpp::Node
{
  public:
    LocalizationNode();
    ~LocalizationNode();

  private:
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_loc_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_slam_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_twist_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tfB_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_odom_;
    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub_;
    rclcpp::Time last_clock_time_;
    void GpsCallBack(const sensor_msgs::msg::NavSatFix::SharedPtr msg_in);
    void ImuCallBack(const sensor_msgs::msg::Imu::SharedPtr msg_in);
    void SlamCallBack(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg_in);
    void TwistCallBack(const geometry_msgs::msg::Twist::SharedPtr msg_in);
    void clockCallback(const rosgraph_msgs::msg::Clock::SharedPtr msg);
    void timer_callback();

    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::PoseWithCovarianceStamped slam_pose_;
    sensor_msgs::msg::Imu imu_pose_;
    sensor_msgs::msg::NavSatFix gps_;
    geometry_msgs::msg::Twist twist_;

    bool pose_received_{false};
  UKF ukf_ ;
  geodetic_converter::GeodeticConverter converter_;
bool gps_init_{false};
  std::tuple<geometry_msgs::msg::TransformStamped, nav_msgs::msg::Odometry> set_oputout(
  double x,double y,double theta,
rclcpp::Time last_clock_time){

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp.sec = last_clock_time.seconds();
    transform.header.stamp.nanosec = last_clock_time.nanoseconds();
    transform.header.frame_id = "odom";
    transform.child_frame_id = "base_link";

    transform.transform.translation.x = x;
    transform.transform.translation.y = y;
    transform.transform.translation.z = 0;

    transform.transform.rotation.x = 0;
    transform.transform.rotation.y = 0;
    transform.transform.rotation.z = theta;
    transform.transform.rotation.w = 1;
    auto odom = nav_msgs::msg::Odometry();
    odom.header.stamp.sec = last_clock_time.seconds();
    odom.header.stamp.nanosec = last_clock_time.nanoseconds();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    // Set position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;

    odom.pose.pose.orientation.x = 0;
    odom.pose.pose.orientation.y = 0;
    odom.pose.pose.orientation.z = theta;
    odom.pose.pose.orientation.w = 1;

    return std::make_tuple(transform, odom);
  };
};