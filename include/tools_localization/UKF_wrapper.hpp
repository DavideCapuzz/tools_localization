//
// Created by davide on 10/25/25.
//

#ifndef TOOLS_LOCALIZATION_UKF_WRAPPER_HPP
#define TOOLS_LOCALIZATION_UKF_WRAPPER_HPP

#include "ukf_defs.hpp"
#pragma once
#include "UKF.hpp"
#include "basic_filter.hpp"
#include <memory>
#include <yaml-cpp/yaml.h>

class UKFwrapper : public FilterBase {
public:
  UKFwrapper(const YAML::Node &config_node) {
    // std::string config_file_path =
    // "/home/davide/ros_ws/wheele/src/tools_localization/config/config.json";
    const std::string config_file_path =
        config_node["config_file_path"].as<std::string>();

    ukf_ = std::make_unique<UKF>(config_file_path);
    // make a dummy state
    StateVec initial_state;
    CovMat initial_covariance;

    initial_covariance.setZero();
    initial_covariance.diagonal() << 0.5, 0.5, 0.5, // positions
        0.1, 0.1, 0.1,                              // velocities
        0.25, 0.25, 0.25,                           // attitude anglesz
        1e-4, 1e-4, 1e-4,                           // accelerometer biases
        1e-4, 1e-4, 1e-4;                           // gyro biases

    initial_state = 1e-3 * Eigen::Matrix<double, N, 1>::Ones();

    //ukf_->initialize(initial_state, initial_covariance);
    ukf_->start_filter(20ms);
  }

  void predict(const double time, const Eigen::VectorXd &u) override {
    // init imu data
    //ImuData imu_dat;
    //imu_dat.measurement_time = time;
    //imu_dat.matrix_form_measurement = u;
    //imu_dat.updateFromMatrix(); // will assign accx, dphix etc members

    //ukf_->read_imu(imu_dat);

    // make an IMU data struct for the filter
    ImuData imu_data;
    ControlInput control_input;

    control_input << u[3], u[4], u[5], u[0], u[1], u[2];

    imu_data.measurement_time = time;
    imu_data.matrix_form_measurement = control_input;
    imu_data.updateFromMatrix(); // updates helpful double members for
    // accelerometer and gyroscope data

    // sleep until the estimator has space in its measurement queues
    if (!wait_until_queue_has_space(true, std::chrono::milliseconds(600),
                                    ukf_)) {
      std::cout<<"IMU queue full; dropping sample\n";
      /*RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "IMU queue full; dropping sample");*/
      return;
                                    };
    ukf_->read_imu(imu_data);
  }

  void update(const double time, const Eigen::VectorXd &z) override {
    // init observation data
    /*Observable observation;
    observation.timestamp = time;
    observation.observation = z;

    Eigen::Matrix<double, Z, 1> MeasVec;
    MeasVec << z[0], z[1], z[2];
    Eigen::Matrix<double, Z, Z> MeasCov;
    MeasCov << z[3], z[4], z[5], z[6], z[7], z[8], z[9], z[10], z[11];
    observation.R = MeasCov;*/

    Observable obs_dat;
    MeasVec meas;
    MeasCov R;

    meas << z[0], z[1], z[2];
    R << z[3], z[4], z[5], z[6], z[7], z[8], z[9], z[10], z[11];

    obs_dat.observation = meas;
    obs_dat.R = R;
    obs_dat.timestamp = time;

    // sleep until the estimator has space in its measurement queues
    if (!wait_until_queue_has_space(false, std::chrono::milliseconds(600),
                                    ukf_)) {
      std::cout<<"GNSS queue full; dropping sample\n";
      /*RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "GNSS queue full; dropping sample");*/
      return;
                                    };
    ukf_->read_gps(obs_dat);
  }
  Eigen::VectorXd get_state() override { return ukf_->get_state(); }

  bool wait_until_queue_has_space(
    bool imu, std::chrono::milliseconds timeout,
    const std::shared_ptr<Estimator> &est) noexcept {
    using clock = std::chrono::steady_clock;
    const auto deadline = clock::now() + timeout;
    auto sleep_ms = std::chrono::milliseconds(10);

    while (clock::now() < deadline) {
      if (!est->queues_full(imu))
        return true;
      std::this_thread::sleep_for(sleep_ms); // backoff
    }
    return false; // timeout
  }

private:
  std::shared_ptr<Estimator> ukf_;
};

#endif // TOOLS_LOCALIZATION_UKF_WRAPPER_HPP
