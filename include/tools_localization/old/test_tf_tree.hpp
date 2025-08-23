#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/message_filter.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include "message_filters/subscriber.h"
using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class TFTest : public rclcpp::Node
{
  public:
    TFTest();
    ~TFTest(){};
void init();
  private:
    
    std::unique_ptr<tf2_ros::Buffer> tf_;
    std::unique_ptr<tf2_ros::TransformListener> tfL_;

  std::unique_ptr<message_filters::Subscriber<sensor_msgs::msg::LaserScan>> scan_filter_sub_;
  std::unique_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>> scan_filter_;
  void laserCallback(
  sensor_msgs::msg::LaserScan::ConstSharedPtr scan);
  
};