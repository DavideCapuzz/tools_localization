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
LocalizationNode::LocalizationNode(const rclcpp::NodeOptions &options) :
Node("LocalizationNode",options)
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
      50ms, std::bind(&LocalizationNode::timer_callback, this));

    // std::string config_file_path = this->get_parameter("config_path").as_string();
    this->declare_parameter("filter_type", "ekf");
    std::string filter_type = this->get_parameter("filter_type").as_string();
    YAML::Node config_node;
    filter_ = filter_factory(filter_type,config_node);
    YAML::Node config_log;
    config_log["out_file"] = "/home/davide/ros_ws/wheele/src/tools_localization/test/data/result_data/result";
    config_log["log_raw_pose"] =  true;
    config_log["log_end_pose"] =  true;
    set_up_debug_log(config_log);
}

LocalizationNode::~LocalizationNode() {
    writer.close();
}

void LocalizationNode::timer_callback()
{
    auto state = filter_->get_state();
    auto [transform, odom] =
        set_oputout(state[0],state[1],state[2],last_clock_time_);
    // auto [transform, odom] =
    //     set_oputout(0,0,0,last_clock_time_);
    geometry_msgs::msg::PointStamped gps_point;
    gps_point.header.stamp = gps_.header.stamp;
    gps_point.header.frame_id = gps_.header.frame_id;  // ENU frame
    gps_point.point.x = state[0];
    gps_point.point.y = state[1];
    gps_point.point.z = 0;

    save_data<geometry_msgs::msg::PointStamped>(gps_point, "/pose_end");

    try {
        RCLCPP_WARN(this->get_logger(), "Transform failed send: %d, %d,%d,%d");
        tf_brodacaster_->sendTransform(transform);
        std::cout<<"of "<<state[0]<<" "<<state[1]<<" "<<state[1]<<"\n";
        std::cout<<"ot "<<odom.pose.pose.position.x<<" "<<odom.pose.pose.position.x<<" "<<odom.pose.pose.position.x<<"\n";
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
        Eigen::VectorXd u(6);
        u << accel_out.vector.x, accel_out.vector.y, accel_out.vector.z,
            gyro_out.vector.x, gyro_out.vector.y, gyro_out.vector.z;
        filter_->predict(imu_pose_.header.stamp.sec + imu_pose_.header.stamp.nanosec * 1e-9, u);
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
        // converter_.initialiseReference(gps_.latitude, gps_.longitude, gps_.altitude);
        gps_init_ = true;
        local_gps_origin_ = {41.9028, 12.4964, 50.0};  // Lat/Lon in deg, Alt in m
    }
  refx::Coordinate3D<refx::lla> target_gps_point(gps_.latitude, gps_.longitude, gps_.altitude);

    auto earthmodel = refx::EarthModelWGS84<double>();

    // This performs a runtime projection of the GPS point onto a flat plane at the origin.
  refx::Coordinate3D<refx::enu> pose_enu =
        refx::frame_transform<refx::enu>(target_gps_point, local_gps_origin_, earthmodel);
    geometry_msgs::msg::PointStamped gps_point;
    gps_point.header.stamp = gps_.header.stamp;
    gps_point.header.frame_id = gps_.header.frame_id;  // ENU frame
    gps_point.point.x = pose_enu.x();
    gps_point.point.y = pose_enu.y();
    gps_point.point.z = pose_enu.z();

    save_data<geometry_msgs::msg::PointStamped>(gps_point, "/pose_raw");

    try {
        geometry_msgs::msg::PointStamped gps_point_transformed = tf_buffer_->transform(
                gps_point,
                "base_link",
                tf2::durationFromSec(0.1)  // timeout
            );
        std::cout<<"i "<<gps_point_transformed.point.x<<" "<<gps_point_transformed.point.y<<" "<<gps_point_transformed.point.z<<std::endl;

        Eigen::VectorXd z(12);
        z << gps_point_transformed.point.x, gps_point_transformed.point.y, gps_point_transformed.point.z,
        gps_.position_covariance.at(0), gps_.position_covariance.at(1),gps_.position_covariance.at(2),
        gps_.position_covariance.at(3), gps_.position_covariance.at(4),gps_.position_covariance.at(5),
        gps_.position_covariance.at(6), gps_.position_covariance.at(7),gps_.position_covariance.at(8);
        filter_->update({},z);
    }
    catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
    }

}

std::tuple<geometry_msgs::msg::TransformStamped, nav_msgs::msg::Odometry> LocalizationNode::set_oputout(
    const double x, const double y, const double theta, const rclcpp::Time & last_clock_time){

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
    odom.header = transform.header;
    odom.child_frame_id = transform.child_frame_id;
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = transform.transform.rotation;

    return {transform, odom};
}

void LocalizationNode::SlamCallBack(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg_in)
{
  slam_pose_ = *msg_in;
}
