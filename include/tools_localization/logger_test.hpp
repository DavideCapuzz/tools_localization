//
// Created by davide on 10/25/25.
//

#ifndef TOOLS_LOCALIZATION_LOGGER_TEST_HPP
#define TOOLS_LOCALIZATION_LOGGER_TEST_HPP

#include <fstream>
#include "basic_filter.hpp"
#include <memory>
#include <yaml-cpp/yaml.h>

class LoggerTest : public FilterBase {
public:
  LoggerTest(const YAML::Node &config_node):
  ImuFile_("/home/davide/ros_ws/wheele/src/tools_localization/test/data/rosbag2_2025_09_28-07_37_13/imu_file.csv"),
  GNSSFile_("/home/davide/ros_ws/wheele/src/tools_localization/test/data/rosbag2_2025_09_28-07_37_13/gnss_file.csv"){
    GNSSFile_<<"timestamp,x,y,z,vx,vy,vz,cov_e_x,cov_e_y,cov_e_z,cov_e_vx,cov_e_vy,cov_e_vz\n";
    ImuFile_<<"timestamp,gyro_x,gyro_y,gyro_z,accel_x,accel_y,accel_z\n";
  }
  ~LoggerTest() override{
    
    if (GNSSFile_.is_open())
      GNSSFile_.close();
    if (ImuFile_.is_open())
      ImuFile_.close();
  }

  void predict(const double time, const Eigen::VectorXd &u) override {
    ImuFile_<<time<<","<<u[3]<<","<<u[4]<<","<<u[5]<<","<<u[0]<<","<<u[1]<<","<<u[2]<<"\n";
  }

  void update(const double time, const Eigen::VectorXd &z) override {
    GNSSFile_<<time<<","<<z[0]<<","<<z[1]<<","<<z[2]<<",,,"<<z[3]<<","<<z[7]<<","<<z[11]<<",,,"<<"\n";
  }
  Eigen::VectorXd get_state() override {
    Eigen::VectorXd out(3);
    out<<0,0,0;
    return out;
  }

private:
  std::ofstream ImuFile_;
  std::ofstream GNSSFile_;
};

#endif // TOOLS_LOCALIZATION_LOGGER_TEST_HPP
