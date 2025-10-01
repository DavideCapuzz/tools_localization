#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "tools_localization/localization_node.hpp"
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "rosbag2_cpp/reader.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <rclcpp/time_source.hpp>
using namespace std::chrono_literals;

class LocalizationNodeTest : public ::testing::Test
{
protected:
    static void SetUpTestSuite()
    {
        rclcpp::init(0, nullptr);
    }

    static void TearDownTestSuite()
    {
        rclcpp::shutdown();
    }

    void SetUp() override
    {
        if (!rclcpp::ok()) {
            SetUpTestSuite();
        }
        // node_ = std::make_shared<LocalizationNode>();

        clock_= std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);


        // Create TF buffer with the clock
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(clock_);
        tf_buffer_->setUsingDedicatedThread(true);
    }


    // rclcpp::Node::SharedPtr node_;
    std::shared_ptr<rclcpp::Clock> clock_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    rclcpp::Time latest_clock_time_{0, 0, RCL_ROS_TIME};
};

TEST_F(LocalizationNodeTest, Loadmcap)
{
    const std::string filename = "/home/davide/ros_ws/wheele/bags/rosbag2_2025_09_28-07_37_13/rosbag2_2025_09_28-07_37_13_0.mcap";
    rosbag2_cpp::Reader reader;
    reader.open(filename);
    // tfBuffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock(), tf2::durationFromSec(30.0));
    while (reader.has_next()) {
        auto bag_msg = reader.read_next();
        if (bag_msg->topic_name == "/clock") {
            auto clock_msg = std::make_shared<rosgraph_msgs::msg::Clock>();
            rclcpp::SerializedMessage serialized_msg(*bag_msg->serialized_data);
            rclcpp::Serialization<rosgraph_msgs::msg::Clock> serialization;
            serialization.deserialize_message(&serialized_msg, clock_msg.get());

        }

        if (bag_msg->topic_name == "/tf") {
            auto tf_msg = std::make_shared<tf2_msgs::msg::TFMessage>();
            rclcpp::SerializedMessage serialized_msg(*bag_msg->serialized_data);
            rclcpp::Serialization<tf2_msgs::msg::TFMessage> serialization;
            serialization.deserialize_message(&serialized_msg, tf_msg.get());

            for (const auto& transform : tf_msg->transforms) {
                //std::cout<<"dynamic from "<<transform.child_frame_id<<" to "<<transform.header.frame_id<<std::endl;
                tf_buffer_->setTransform(transform, "mcap_loader", false);
            }
        } else if (bag_msg->topic_name == "/tf_static") {
            auto tf_msg = std::make_shared<tf2_msgs::msg::TFMessage>();
            rclcpp::SerializedMessage serialized_msg(*bag_msg->serialized_data);
            rclcpp::Serialization<tf2_msgs::msg::TFMessage> serialization;
            serialization.deserialize_message(&serialized_msg, tf_msg.get());

            for (const auto& transform : tf_msg->transforms) {
                //std::cout<<"static from "<<transform.child_frame_id<<" to "<<transform.header.frame_id<<std::endl;

                tf_buffer_->setTransform(transform, "mcap_loader", true);
            }
        }

        if (bag_msg->topic_name == "/navsat") {
            rclcpp::Serialization<sensor_msgs::msg::NavSatFix> serializer;
            sensor_msgs::msg::NavSatFix msg;
            rclcpp::SerializedMessage serialized_msg(*bag_msg->serialized_data);
            serializer.deserialize_message(&serialized_msg, &msg);
        }

        if (bag_msg->topic_name == "/imu") {
            rclcpp::Serialization<sensor_msgs::msg::Imu> serializer;
            sensor_msgs::msg::Imu msg;
            rclcpp::SerializedMessage serialized_msg(*bag_msg->serialized_data);
            serializer.deserialize_message(&serialized_msg, &msg);

            msg.header.frame_id = "imu";
            geometry_msgs::msg::Vector3Stamped accel_in, accel_out;
            accel_in.header = msg.header;
            accel_in.vector = msg.linear_acceleration;
            try {
                tf_buffer_->transform(accel_in, accel_out, "base_link", tf2::durationFromSec(0.1));

                geometry_msgs::msg::Vector3Stamped gyro_in, gyro_out;
                gyro_in.header = msg.header;
                gyro_in.vector = msg.angular_velocity;

                tf_buffer_->transform(gyro_in, gyro_out, "base_link", tf2::durationFromSec(0.1));
            //     ukf_.read_imu({
            //     accel_out.vector.x,
            //     accel_out.vector.y,
            //     accel_out.vector.z,
            //     gyro_out.vector.x,
            //     gyro_out.vector.y,
            //     gyro_out.vector.z,
            //     imu_pose_.header.stamp.sec + imu_pose_.header.stamp.nanosec * 1e-9
            // });
            }
            catch (tf2::TransformException &ex) {
                std::cout<< "Transform failed:"<<ex.what()<<"\n";
            }
        }
    }
}

// TEST_F(LocalizationNodeTest, GpsCallbackUpdatesState)
// {
//     auto gps_msg = std::make_shared<sensor_msgs::msg::NavSatFix>();
//     gps_msg->header.stamp = node_->now();
//     gps_msg->header.frame_id = "map";
//     gps_msg->latitude = 45.0;
//     gps_msg->longitude = 9.0;
//     gps_msg->altitude = 200.0;
//     gps_msg->position_covariance = {1,0,0,0,1,0,0,0,1};
//
//     // Call directly (faster than publishing)
//     // ASSERT_NO_THROW(node_->GpsCallBack(gps_msg));
// }
//
// TEST_F(LocalizationNodeTest, ImuCallbackDoesNotThrow)
// {
//     auto imu_msg = std::make_shared<sensor_msgs::msg::Imu>();
//     imu_msg->header.stamp = node_->now();
//     imu_msg->header.frame_id = "base_link";
//     imu_msg->linear_acceleration.x = 0.1;
//     imu_msg->angular_velocity.z = 0.05;
//
//     // ASSERT_NO_THROW(node_->ImuCallBack(imu_msg));
// }

// TEST_F(LocalizationNodeTest, TimerCallbackRuns)
// {
//     //ASSERT_NO_THROW(node_->timer_callback());
// }



int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}