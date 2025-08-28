//
// Created by davide on 8/22/25.
//

#ifndef WHEELE_BASICFILTER_HPP
#define WHEELE_BASICFILTER_HPP

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
using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class BasiFilter
{
  public:
    BasiFilter(){};
    ~BasiFilter(){};


    std::tuple<geometry_msgs::msg::TransformStamped, nav_msgs::msg::Odometry> update(
geometry_msgs::msg::Twist twist,
rclcpp::Time last_clock_time){
      theta_ += (twist.angular.z * dt_ * k_theta_);
      x_ += twist.linear.x * cos(theta_) * dt_ * k_speed_;
      y_ += twist.linear.x * sin(theta_) * dt_ * k_speed_;

      geometry_msgs::msg::TransformStamped transform;
      transform.header.stamp.sec = last_clock_time.seconds();
      transform.header.stamp.nanosec = last_clock_time.nanoseconds();
      transform.header.frame_id = "odom";
      transform.child_frame_id = "base_link";

      transform.transform.translation.x = x_;
      transform.transform.translation.y = y_;
      transform.transform.translation.z = 0;

      transform.transform.rotation.x = 0;
      transform.transform.rotation.y = 0;
      transform.transform.rotation.z = theta_;
      transform.transform.rotation.w = 1;
      auto odom = nav_msgs::msg::Odometry();
      odom.header.stamp.sec = sec_;
      odom.header.stamp.nanosec = nanosec_;
      odom.header.frame_id = "odom";
      odom.child_frame_id = "base_link";
      // Set position
      odom.pose.pose.position.x = x_;
      odom.pose.pose.position.y = y_;
      odom.pose.pose.position.z = 0.0;

      odom.pose.pose.orientation.x = 0;
      odom.pose.pose.orientation.y = 0;
      odom.pose.pose.orientation.z = theta_;
      odom.pose.pose.orientation.w = 1;

      return std::make_tuple(transform, odom);
    };
private:
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

#endif //WHEELE_BASICFILTER_HPP