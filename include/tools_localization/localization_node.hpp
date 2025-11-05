#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <chrono>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <string>
#include <sys/time.h>
#include <thread>
#include <vector>

#include "estimator_interface.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <nav_msgs/msg/odometry.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include "factory_filter.hpp"
// #include "geodetic_utils/geodetic_conv.hpp"
#include "gtest/gtest_prod.h"
#include <cmath>
#include <iostream>
#include <refx/geometry.h> // for Vector3D, Coordinate3D, Rotation and YawPitchRoll
#include <refx/transformations.h> // frame_cast, frame_transform

#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rosbag2_storage/storage_options.hpp>

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class LocalizationNode final : public rclcpp::Node {
public:
  LocalizationNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~LocalizationNode();

private:
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_loc_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      sub_slam_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_twist_;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_brodacaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_odom_;

  void GpsCallBack(const sensor_msgs::msg::NavSatFix::SharedPtr msg_in);
  void ImuCallBack(const sensor_msgs::msg::Imu::SharedPtr msg_in);
  void SlamCallBack(
      const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg_in);
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

  std::tuple<geometry_msgs::msg::TransformStamped, nav_msgs::msg::Odometry>
  set_oputout(const double x, const double y, const double theta,
              const rclcpp::Time &last_clock_time);

    std::map<std::string, bool> sett_log_;
  void set_up_debug_log(const YAML::Node& config_log) {
      rosbag2_storage::StorageOptions storage_options;
      // storage_options.uri = "/home/davide/ros_ws/wheele/src/tools_localization/test/data/result_data/result";
      storage_options.uri = config_log["out_file"].as<std::string>();
      storage_options.storage_id = "mcap";

      rosbag2_cpp::ConverterOptions converter_options;
      converter_options.input_serialization_format = "cdr";
      converter_options.output_serialization_format = "cdr";

      writer.open(storage_options, converter_options);

      if (config_log["log_raw_pose"].as<bool>()) {
        sett_log_.insert({"raw_pose", true});
        setup_log_topic("/pose_raw", "geometry_msgs/msg/PointStamped");
      } else {
        sett_log_.insert({"raw_pose", false});
      }
    if (config_log["log_end_pose"].as<bool>()) {
      sett_log_.insert({"end_pose", true});
      setup_log_topic("/pose_end", "geometry_msgs/msg/PointStamped");
    } else {
      sett_log_.insert({"end_pose", false});
    }
    if (config_log["log_acc_transformed"].as<bool>()) {
      sett_log_.insert({"acc_transformed", true});
      setup_log_topic("/acc_transformed", "geometry_msgs/msg/Vector3Stamped");
    } else {
      sett_log_.insert({"acc_transformed", false});
    }
    if (config_log["log_gyro_transformed"].as<bool>()) {
      sett_log_.insert({"gyro_transformed", true});
      setup_log_topic("/gyro_transformed", "geometry_msgs/msg/Vector3Stamped");
    } else {
      sett_log_.insert({"gyro_transformed", false});
    }

    }

    void setup_log_topic(const std::string & msg_name, const std::string & topic) {
      rosbag2_storage::TopicMetadata topic_metadata;
      topic_metadata.name = msg_name;
      topic_metadata.type = topic;
      topic_metadata.serialization_format = "cdr";
      writer.create_topic(topic_metadata);
    }

  template<typename T>
  void save_data(const T & in_data, const std::string & topic_name, const rclcpp::Time & time) {
    rclcpp::SerializedMessage serialized_msg;
    const rclcpp::Serialization<geometry_msgs::msg::PointStamped> serializer;
    serializer.serialize_message(&in_data, &serialized_msg);

    // Prepare rosbag message
    const auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
    bag_message->topic_name = topic_name;
    bag_message->recv_timestamp = rclcpp::Clock().now().nanoseconds();

    // Copy serialized buffer
    auto data = std::make_shared<rcutils_uint8_array_t>();
    auto src = serialized_msg.get_rcl_serialized_message();

    data->allocator = src.allocator;
    data->buffer_length = src.buffer_length;
    data->buffer_capacity = src.buffer_capacity;
    data->buffer = static_cast<uint8_t *>(malloc(src.buffer_length));
    memcpy(data->buffer, src.buffer, src.buffer_length);

    bag_message->serialized_data = data;

    // Write the message to the bag
    writer.write(bag_message);
  }

  FRIEND_TEST(LocalizationNodeTest, Loadmcap);
};
