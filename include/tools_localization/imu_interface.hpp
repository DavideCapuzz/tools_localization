#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/odometry.hpp>
using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class ImuInterface : public rclcpp::Node
{
  public:
    ImuInterface();
    ~ImuInterface();

  private:
    void timer_callback();
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;
};