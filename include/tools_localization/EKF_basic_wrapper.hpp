//
// Created by davide on 10/25/25.
//

#ifndef TOOLS_LOCALIZATION_EKF_BASIC_WRAPPER_HPP
#define TOOLS_LOCALIZATION_EKF_BASIC_WRAPPER_HPP

#pragma once
#include "basic_filter.hpp"
#include "EKF_basic/EKF_basic.hpp"
#include <yaml-cpp/yaml.h>

class EKF : public FilterBase {
    public:
    EKF(const YAML::Node& config_node){
        // std::string config_file_path = "/home/davide/ros_ws/wheele/src/tools_localization/config/config.json";
        const std::string config_file_path = config_node["config_file_path"].as<std::string>();
        double xy_obs_noise_std = 5.0; // Standard deviation of observation noise of x and y in meters
        double yaw_rate_noise_std = 0.02; // Standard deviation of yaw rate in rad/s
        double forward_velocity_noise_std = 0.3; // Standard deviation of forward velocity in m/s

        // Prepare initial estimate and its error covariance
        double initial_yaw_std = M_PI;
        double initial_yaw = 0;

        Eigen::Vector3d x(0,0, initial_yaw);
        ekf_.initialize(x, xy_obs_noise_std, yaw_rate_noise_std, forward_velocity_noise_std, initial_yaw_std);
    }
    void predict(const double time, const Eigen::VectorXd &u) override {
        // double dt = time_ - time;
        double dt = 0.02;
        time_ = time;
        if (dt<2) {

            v_ += dt*u[0];
            std::cout<<"filter"<<v_<< "  "<< dt<<"  " <<u[0]<<std::endl;

            Eigen::Vector2d u_in(v_, u[5]);
            // Because velocity and yaw rate are multiplied with `dt` in the state transition function,
            // its noise covariance must be multiplied with `dt**2.`
            //Eigen::Matrix3d R_ = R * (dt * dt);
            // Propagate!
            ekf_.propagate(u_in, dt);
        }
    }
    void update(const double time, const Eigen::VectorXd &z) override {
        Eigen::Vector2d z_in(z[0], z[1]);
        // Update!
        ekf_.update(z_in);
    }
    Eigen::VectorXd get_state() override {

        return Eigen::VectorXd(ekf_.getState());
    }

    private:
    ExtendedKalmanFilter ekf_ ;
    double v_{0};
    double time_{0};
};


#endif //TOOLS_LOCALIZATION_EKF_BASIC_WRAPPER_HPP