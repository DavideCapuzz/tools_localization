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
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_ros/buffer.h>
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include "UKF/UKF.hpp"
#include "geodetic_utils/geodetic_conv.hpp"
#include "gtest/gtest_prod.h"

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
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_brodacaster_;
  // std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  void GpsCallBack(const sensor_msgs::msg::NavSatFix::SharedPtr msg_in);
  void ImuCallBack(const sensor_msgs::msg::Imu::SharedPtr msg_in);
  void SlamCallBack(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg_in);
  void TwistCallBack(const geometry_msgs::msg::Twist::SharedPtr msg_in);
  void clockCallback(const rosgraph_msgs::msg::Clock::SharedPtr msg);
  void timer_callback();

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_odom_;
    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub_;
    rclcpp::Time last_clock_time_;

    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::PoseWithCovarianceStamped slam_pose_;
    sensor_msgs::msg::Imu imu_pose_;
    sensor_msgs::msg::NavSatFix gps_;
    geometry_msgs::msg::Twist twist_;

    bool pose_received_{false};
  UKF ukf_ ;
  geodetic_converter::GeodeticConverter converter_;
  bool gps_init_{false};

  double north_{0.0}, east_{0.0}, up_{0.0}, sec_{0.0};

  std::tuple<geometry_msgs::msg::TransformStamped, nav_msgs::msg::Odometry> set_oputout(
  double x,double y,double theta,
rclcpp::Time last_clock_time){

    // --- Convert yaw to quaternion
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta);

    // --- Create TransformStamped
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = last_clock_time;
    transform.header.frame_id = "odom";
    transform.child_frame_id = "base_link";
    transform.transform.translation.x = x;
    transform.transform.translation.y = y;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation = tf2::toMsg(q);

    // --- Create Odometry
    nav_msgs::msg::Odometry odom;
    odom.header = transform.header;  // copy timestamp + frame_id
    odom.child_frame_id = transform.child_frame_id;
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = transform.transform.rotation;

    // Optional: Add velocity and covariance fields if needed
    // odom.twist.twist.linear.x = vx;
    // odom.pose.covariance = { ... };

    return {transform, odom};
  };
  // friend class LocalizationNodeTest;
  FRIEND_TEST(LocalizationNodeTest, Loadmcap);
};