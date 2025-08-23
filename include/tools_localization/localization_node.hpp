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

#include "tools_localization/filters/basicfilter.hpp"
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

    BasiFilter filter_;

    bool pose_received_{false};
    size_t count_;
    int32_t sec_;
    uint32_t nanosec_;
    double x_{0.};
    double y_{0.};
    double theta_{0.};
    double k_theta_{1};
    double k_speed_{1};
    double dt_{0.1};
};