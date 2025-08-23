// Copyright 2019 coderkarl. Subject to the BSD license.

#include "tools_localization/interfaces/gps_interface.hpp"
#include <geometry_msgs/msg/point_stamped.h>
#include <math.h>
#include <boost/math/special_functions/round.hpp>
#include <algorithm>

/**********************************************************************
* Obstacle Avoidance using a nav_msgs/OccupacyGrid and A* path planning
* 
* Subscribe to /scan and Publish OccupacyGrid /costmap, Publish Path /path
* 
**********************************************************************/

using std::placeholders::_1;

//Constructor
GPSInterface::GPSInterface(std::shared_ptr<GPSCapturer> capturer):
    Node("GPSInterface"), capturer_(capturer)
{
    pub_gps_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/navsat", 10);
    timer_ = this->create_wall_timer(
    10ms, std::bind(&GPSInterface::timer_callback, this));
}

GPSInterface::~GPSInterface(){}

void GPSInterface::timer_callback()
{
  sensor_msgs::msg::NavSatFix message = capturer_->getLatestGPS();
  message.header.frame_id="base_link";
  pub_gps_->publish(message);
}

GPSCapturer::GPSCapturer(const std::string device, int baud_rate)
: device_(device), baud_rate_(baud_rate), serial_(io_, device), capturing_(true)
{
  init_gps();
}

GPSCapturer::GPSCapturer()
: serial_(io_, device_), capturing_(true)
{
  init_gps();
}

GPSCapturer::~GPSCapturer()
{
    stop();
}

void GPSCapturer::init_gps(){  
  serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
  serial_.set_option(boost::asio::serial_port_base::character_size(8));
  serial_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
  serial_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
}

void GPSCapturer::captureLoop(){
  std::vector<char> buf;
  char b_in[120];
  boost::system::error_code error;
  Ublox M8_Gps;
  while (true) {
    size_t len = serial_.read_some(boost::asio::buffer(b_in), error);
    

    if (error) {
      std::cerr << "Error reading: " << error.message() << std::endl;
      break;
    }

    if(b_in[0]=='\n')
    {
      std::string str(buf.begin(), buf.end());
      if (M8_Gps.encode(str)) {
        gps_out_.altitude = M8_Gps.altitude;
        gps_out_.latitude = M8_Gps.latitude;
        gps_out_.longitude = M8_Gps.longitude; 
        // std::cout<<" alt "<< M8_Gps.altitude<<" lat "<< M8_Gps.latitude<<" lon "<< M8_Gps.longitude<<" sts "<< M8_Gps.sats_in_use<<"\n";
      }
      buf.clear();
    }
    else{
      buf.push_back(b_in[0]);
    }
  }
}

void GPSCapturer::stop()
{
    capturing_.store(false);
}

sensor_msgs::msg::NavSatFix GPSCapturer::getLatestGPS()
{
    std::lock_guard<std::mutex> lock(frame_mutex_);    
    return gps_out_;
}
      
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto capturer = std::make_shared<GPSCapturer>();

  auto publisher = std::make_shared<GPSInterface>(capturer);

  std::thread capture_thread(&GPSCapturer::captureLoop, capturer);

  rclcpp::spin(publisher);

  capturer->stop();
  if (capture_thread.joinable()) {
      capture_thread.join();
  }
}
