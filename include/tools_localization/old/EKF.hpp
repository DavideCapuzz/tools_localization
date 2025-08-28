//
// Created by davide on 8/22/25.
//

#ifndef WHEELE_EKF_HPP
#define WHEELE_EKF_HPP

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

#include <kalman/LinearizedMeasurementModel.hpp>  // GPS is linear here
#include <kalman/LinearizedSystemModel.hpp>  // GPS is linear here
#include <kalman/ExtendedKalmanFilter.hpp>
#include <kalman/UnscentedKalmanFilter.hpp>

using namespace std::chrono_literals;

class EKF
{
  public:
    EKF():
	pm_(-10, -10, 30, 75)
	{
		state_x_.setZero();
    	predictor_.init(state_x_);
    	ekf_.init(state_x_);
    	init_output();
	};
    ~EKF(){};


    std::tuple<geometry_msgs::msg::TransformStamped, nav_msgs::msg::Odometry> update(
geometry_msgs::msg::Twist twist,
rclcpp::Time last_clock_time){
    	input_u_.v() = 0;
    	input_u_.dtheta() = 0;

    	auto x_pred = predictor_.predict(sys_, input_u_);
    	auto x_ekf = ekf_.predict(sys_, input_u_);
    	OrientationMeasurement orientation;
    	orientation.theta();
    	x_ekf = ekf_.update(om_, orientation);
      return prepare_output(x_,y_, theta_, last_clock_time.seconds(),last_clock_time.nanoseconds());
    };
private:

	std::tuple<geometry_msgs::msg::TransformStamped, nav_msgs::msg::Odometry> prepare_output(
double x, double y, double theta, double sec, double nanosec){

		transform_.header.stamp.sec = sec;
		transform_.header.stamp.nanosec = nanosec;
		transform_.transform.translation.x = x;
		transform_.transform.translation.y = y;
		transform_.transform.rotation.z = theta;
		odom_.header.stamp.sec = sec;
		odom_.header.stamp.nanosec = nanosec;
		odom_.pose.pose.position.x = x;
		odom_.pose.pose.position.y = y;
		odom_.pose.pose.orientation.z = theta;

		return std::make_tuple(transform_, odom_);
	};

	void init_output() {
		transform_.header.stamp.sec = 0.0;
		transform_.header.stamp.nanosec = 0.0;
		transform_.header.frame_id = "odom";
		transform_.child_frame_id = "base_link";

		transform_.transform.translation.x = 0.0;
		transform_.transform.translation.y = 0.0;
		transform_.transform.translation.z = 0;

		transform_.transform.rotation.x = 0;
		transform_.transform.rotation.y = 0;
		transform_.transform.rotation.z = 0.0;
		transform_.transform.rotation.w = 1;
		odom_.header.stamp.sec = 0.0;
		odom_.header.stamp.nanosec = 0.0;
		odom_.header.frame_id = "odom";
		odom_.child_frame_id = "base_link";
		// Set position
		odom_.pose.pose.position.x = 0.0;
		odom_.pose.pose.position.y = 0.0;
		odom_.pose.pose.position.z = 0.0;

		odom_.pose.pose.orientation.x = 0;
		odom_.pose.pose.orientation.y = 0;
		odom_.pose.pose.orientation.z = 0;
		odom_.pose.pose.orientation.w = 1;
	}
	
	size_t count_;
    int32_t sec_;
    uint32_t nanosec_;
    double x_{0.};
    double y_{0.};
    double theta_{0.};
    double k_theta_{1};
    double k_speed_{1};
    double dt_{0.1};

	geometry_msgs::msg::TransformStamped transform_;
	nav_msgs::msg::Odometry odom_;
	State state_x_;
	// System
	SystemModel sys_;
	// Control input
	Control input_u_;

	Kalman::ExtendedKalmanFilter<State> predictor_;
	// Extended Kalman Filter
	Kalman::ExtendedKalmanFilter<State> ekf_;

	PositionModel pm_;
	OrientationModel om_;
};

#endif //WHEELE_EKF_HPP