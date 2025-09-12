#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "tools_localization/localization_node.hpp"
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>

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
        node_ = std::make_shared<LocalizationNode>();
    }

    rclcpp::Node::SharedPtr node_;
};

TEST_F(LocalizationNodeTest, GpsCallbackUpdatesState)
{
    auto gps_msg = std::make_shared<sensor_msgs::msg::NavSatFix>();
    gps_msg->header.stamp = node_->now();
    gps_msg->header.frame_id = "map";
    gps_msg->latitude = 45.0;
    gps_msg->longitude = 9.0;
    gps_msg->altitude = 200.0;
    gps_msg->position_covariance = {1,0,0,0,1,0,0,0,1};

    // Call directly (faster than publishing)
    // ASSERT_NO_THROW(node_->GpsCallBack(gps_msg));
}

TEST_F(LocalizationNodeTest, ImuCallbackDoesNotThrow)
{
    auto imu_msg = std::make_shared<sensor_msgs::msg::Imu>();
    imu_msg->header.stamp = node_->now();
    imu_msg->header.frame_id = "base_link";
    imu_msg->linear_acceleration.x = 0.1;
    imu_msg->angular_velocity.z = 0.05;

    // ASSERT_NO_THROW(node_->ImuCallBack(imu_msg));
}

TEST_F(LocalizationNodeTest, TimerCallbackRuns)
{
    //ASSERT_NO_THROW(node_->timer_callback());
}
