#include <iostream>
#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>
#include "tools_localization/ublox_parser.hpp"
#include "tools_localization/gps_interface.hpp"

// void readData(boost::asio::serial_port& serial) {
//   std::vector<char> buf;
//   char b_in[120];
//   boost::system::error_code error;
//   Ublox M8_Gps;
//   float gpsArray[4] = {0, 0, 0, 0};
//   while (true) {
//     size_t len = serial.read_some(boost::asio::buffer(b_in), error);
    

//     if (error) {
//       std::cerr << "Error reading: " << error.message() << std::endl;
//       break;
//     }

//     if(b_in[0]=='\n')
//     {
//       std::string str(buf.begin(), buf.end());
//       std::cout<<str<<"\n";
//       // if (M8_Gps.encode(str)) {
//       //   gpsArray[0] = M8_Gps.altitude;
//       //   gpsArray[1] = M8_Gps.latitude;
//       //   gpsArray[2] = M8_Gps.longitude; 
//       //   gpsArray[3] = M8_Gps.sats_in_use;
//       //   std::cout<<" alt "<< M8_Gps.altitude<<" lat "<< M8_Gps.latitude<<" lon "<< M8_Gps.longitude<<" sts "<< M8_Gps.sats_in_use<<"\n";

//       // }
//       buf.clear();
//     }
//     else{
//       buf.push_back(b_in[0]);
//     }
//   } 
// }
// int main() {
//     const char* device = "/dev/ttyUSB0"; // Replace with your device
//     try {
//         boost::asio::io_service io;
//         boost::asio::serial_port serial(io, device);
//         serial.set_option(boost::asio::serial_port_base::baud_rate(9600));
//         serial.set_option(boost::asio::serial_port_base::character_size(8));
//         serial.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
//         serial.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));

//         readData(serial);
//     } catch (const std::exception& e) {
//         std::cerr << "Exception: " << e.what() << std::endl;
//     }

//     return 0;
// }

int main() {    
  auto capturer = std::make_shared<GPSCapturer>();

  std::thread capture_thread(&GPSCapturer::captureLoop, capturer);


  capturer->stop();
  if (capture_thread.joinable()) {
      capture_thread.join();
  }
}