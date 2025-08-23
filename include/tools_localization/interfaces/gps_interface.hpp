#include <chrono>
#include <functional>
#include <memory>
#include <string>


#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>

#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include "tools_localization/interfaces/ublox_parser.hpp"
using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class GPSCapturer
{
public:
	GPSCapturer(const std::string device, int baud_rate);

  GPSCapturer();

	~GPSCapturer();

  void captureLoop();
  void stop();
  sensor_msgs::msg::NavSatFix getLatestGPS(); 
  void init_gps();

private:
  const std::string device_ = "/dev/ttyUSB0"; // Replace with your device
  int baud_rate_ = 9600;
  
  boost::asio::io_service io_;
  boost::asio::serial_port serial_;

  sensor_msgs::msg::NavSatFix gps_out_;

  std::mutex frame_mutex_;
  std::atomic<bool> capturing_;
};


class GPSInterface : public rclcpp::Node
{
  public:
    GPSInterface(std::shared_ptr<GPSCapturer> capturer);
    ~GPSInterface();

  private:
    void timer_callback();
    sensor_msgs::msg::NavSatFix gps_out_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<GPSCapturer> capturer_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_gps_;
};