// Copyright 2019 coderkarl. Subject to the BSD license.

#include "tools_localization/localization_node.hpp"
// #include "nav_sim/AvoidObsCommon.h"
#include <geometry_msgs/msg/point_stamped.h>
#include <math.h>
#include <boost/math/special_functions/round.hpp>
#include <algorithm>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_ros/buffer.h>
/**********************************************************************
 * Obstacle Avoidance using a nav_msgs/OccupacyGrid and A* path planning
 *
 * Subscribe to /scan and Publish OccupacyGrid /costmap, Publish Path /path
 *
 **********************************************************************/

using std::placeholders::_1;

// Constructor
LocalizationNode::LocalizationNode() : Node("LocalizationNode")
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
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock(), tf2::durationFromSec(30.0));
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_brodacaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  publisher_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
  timer_ = this->create_wall_timer(
      100ms, std::bind(&LocalizationNode::timer_callback, this));


    // std::string config_file_path = this->get_parameter("config_path").as_string();
    std::string config_file_path = "/home/davide/ros_ws/wheele/src/tools_localization/config/config.json";
    ukf_.configure(config_file_path);
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

    ukf_.initialize(initial_state, initial_covariance);
}

LocalizationNode::~LocalizationNode() {}

void LocalizationNode::timer_callback()
{
    auto state = ukf_.get_state();
    // auto [transform, odom] =
    //     set_oputout(state[0],state[1],state[2],last_clock_time_);
    auto [transform, odom] =
        set_oputout(0,0,0,last_clock_time_);
    try {
        RCLCPP_WARN(this->get_logger(), "Transform failed send: %d, %d,%d,%d");
    tf_brodacaster_->sendTransform(transform);

        publisher_odom_->publish(odom);
    }
    catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Transform failed send: %s", ex.what());
    }

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
  imu_pose_.header.frame_id = "imu";
    geometry_msgs::msg::Vector3Stamped accel_in, accel_out;
    accel_in.header = imu_pose_.header;
    accel_in.vector = imu_pose_.linear_acceleration;
    try {
        tf_buffer_->transform(accel_in, accel_out, "base_link", tf2::durationFromSec(0.1));

        geometry_msgs::msg::Vector3Stamped gyro_in, gyro_out;
        gyro_in.header = imu_pose_.header;
        gyro_in.vector = imu_pose_.angular_velocity;

        tf_buffer_->transform(gyro_in, gyro_out, "base_link", tf2::durationFromSec(0.1));
        ukf_.read_imu({
        accel_out.vector.x,
        accel_out.vector.y,
        accel_out.vector.z,
        gyro_out.vector.x,
        gyro_out.vector.y,
        gyro_out.vector.z,
        imu_pose_.header.stamp.sec + imu_pose_.header.stamp.nanosec * 1e-9
    });
    }
    catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
    }
}

void LocalizationNode::GpsCallBack(const sensor_msgs::msg::NavSatFix::SharedPtr msg_in){
  gps_ = *msg_in;
    gps_.header.frame_id = "gps";
    double sec = gps_.header.stamp.sec + gps_.header.stamp.nanosec;
    if (!gps_init_) {
        converter_.initialiseReference(gps_.latitude, gps_.longitude, gps_.altitude);
        gps_init_ = true;
    }
    geometry_msgs::msg::PointStamped gps_point;
    gps_point.header.stamp = gps_.header.stamp;
    gps_point.header.frame_id = gps_.header.frame_id;  // ENU frame
    converter_.geodetic2Enu(
        gps_.latitude, gps_.longitude, gps_.altitude,
        &gps_point.point.x, &gps_point.point.y , &gps_point.point.z);


    try {
        geometry_msgs::msg::PointStamped gps_point_transformed = tf_buffer_->transform(
                gps_point,
                "base_link",
                tf2::durationFromSec(0.1)  // timeout
            );
        // double dn = (north - north_)/(sec - sec_);
        // double de = (east - east_)/(sec - sec_);
        // double du = (up - up_)/(sec - sec_);
        // north_ = north;
        // east_ = east;
        // up_ = up;
        sec_ = sec;
        Eigen::Matrix<double, Z, 1> MeasVec;
        MeasVec << gps_point_transformed.point.x, gps_point_transformed.point.y, gps_point_transformed.point.z;
        Eigen::Matrix<double, Z, Z> MeasCov;
        MeasCov << gps_.position_covariance.at(0), gps_.position_covariance.at(1),gps_.position_covariance.at(2),
        gps_.position_covariance.at(3), gps_.position_covariance.at(4),gps_.position_covariance.at(5),
        gps_.position_covariance.at(6), gps_.position_covariance.at(7),gps_.position_covariance.at(8);

        ukf_.read_gps({static_cast<double>(imu_pose_.header.stamp.sec),
            MeasVec,
            MeasCov
        });
    }
    catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
    }

}

void LocalizationNode::SlamCallBack(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg_in)
{
  slam_pose_ = *msg_in;
  pose_received_ = true;
}
