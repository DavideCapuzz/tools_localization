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

#include "factory_filter.hpp"
// #include "geodetic_utils/geodetic_conv.hpp"
#include "gtest/gtest_prod.h"
#include <iostream>
#include <cmath>
#include <refx/geometry.h>  // for Vector3D, Coordinate3D, Rotation and YawPitchRoll
#include <refx/transformations.h> // frame_cast, frame_transform

#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/reader.hpp>

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class LocalizationNode final : public rclcpp::Node
{
  public:
    LocalizationNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~LocalizationNode();
  private:
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_loc_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_slam_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_twist_;
    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_brodacaster_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_odom_;

    void GpsCallBack(const sensor_msgs::msg::NavSatFix::SharedPtr msg_in);
    void ImuCallBack(const sensor_msgs::msg::Imu::SharedPtr msg_in);
    void SlamCallBack(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg_in);
    void TwistCallBack(const geometry_msgs::msg::Twist::SharedPtr msg_in);
    void clockCallback(const rosgraph_msgs::msg::Clock::SharedPtr msg);
    void timer_callback();

    std::shared_ptr<FilterBase> filter_;

    rclcpp::Time last_clock_time_;
    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::PoseWithCovarianceStamped slam_pose_;
    sensor_msgs::msg::Imu imu_pose_;
    sensor_msgs::msg::NavSatFix gps_;
    geometry_msgs::msg::Twist twist_;

    bool gps_init_{false};
    refx::Coordinate3D<refx::lla> local_gps_origin_;

    rosbag2_cpp::Writer writer;

    std::tuple<geometry_msgs::msg::TransformStamped, nav_msgs::msg::Odometry> set_oputout(
    const double x, const double y, const double theta, const rclcpp::Time & last_clock_time);

  FRIEND_TEST(LocalizationNodeTest, Loadmcap);
};