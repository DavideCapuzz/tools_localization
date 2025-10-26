//
// Created by davide on 10/25/25.
//

#ifndef TOOLS_LOCALIZATION_UKF_WRAPPER_HPP
#define TOOLS_LOCALIZATION_UKF_WRAPPER_HPP

#pragma once
#include "basic_filter.hpp"
#include "UKF/UKF.hpp"

class UKFwrapper : public FilterBase {
    public:
    UKFwrapper(){
        std::string config_file_path = "/home/davide/ros_ws/wheele/src/tools_localization/config/config.json";
        ukf_.configure(config_file_path);
        // make a dummy state
        StateVec initial_state;
        CovMat initial_covariance;

        initial_covariance.setZero();
        initial_covariance.diagonal() <<
        0.5, 0.5, 0.5,   // positions
        0.1, 0.1, 0.1,   // velocities
        0.25, 0.25, 0.25, // attitude anglesz
        1e-4, 1e-4, 1e-4,   // accelerometer biases
        1e-4, 1e-4, 1e-4;   // gyro biases

        initial_state = 1e-3 * Eigen::Matrix<double, N, 1>::Ones();

        ukf_.initialize(initial_state, initial_covariance);
    }
    void predict(const double time, const Eigen::VectorXd &u) override {
        ukf_.read_imu({u[0], u[1], u[2], u[3], u[4], u[5], time});
    }
    void update(const double time, const Eigen::VectorXd &z) override {
        Eigen::Matrix<double, Z, 1> MeasVec;
        MeasVec << z[0],z[1],z[2];
        Eigen::Matrix<double, Z, Z> MeasCov;
        MeasCov <<
            z[3], z[4], z[5],
            z[6], z[7], z[8],
            z[9], z[10], z[11];
        ukf_.read_gps({time,
            MeasVec,
            MeasCov
        });
    }
    Eigen::VectorXd get_state() override {
        return ukf_.get_state();
    }

    private:
    UKF ukf_ ;
};


#endif //TOOLS_LOCALIZATION_UKF_WRAPPER_HPP