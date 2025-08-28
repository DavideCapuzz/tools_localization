#pragma once

#include "tools_localization/filters/IMU_Matrices.hpp"
// #include "measurement_handler.hpp"

#include <string>
#include <queue>

#include <Eigen/Dense>
#include <nlohmann/json.hpp>

using Matrix = Eigen::MatrixXd;
using Vector = Eigen::VectorXd;
using json = nlohmann::json;

// the IMU model will be responsible for seamless production of the
// time updated state

// configuations will be allowed for defining the level of simplification
// and to specify the expected process noise characteristics

// this script will produce discrete state estimates in PVA according to
// its configuration

class IMU {
    public:
        // generic constructor definition
        // we will want to add definitions for alignment procedures soon
        IMU(const std::string configs_path);

        // provide an external solution for the kickoff of the IMU
        void set_initialization(ImuStateVector& initial_solution,
                                ImuCovariance& initial_covariance,
                                ImuData& initial_measurement);

        // process IMU measurements
        void perform_time_update(ImuData imu_measurements);


    private:
        // basic app configs, state vector size
        std::string _configuration_file_path;
        std::string _measurements_file_path;  // the path to the pre-recorded messages
        Config _config;  // defined in IMU matrices.hpp

        bool _solution_initialized;
        uint _num_states;

        // solutions and points of linearization
        ImuStateVector _nominal_states;  // points of linearization of the state vector
        ImuStateVector _delta_states;  // the perturbation from the linearization points set as the _nominal_state_vector
        ImuCovariance _state_covariance;
        ImuData _nominal_measurements{};  // points of linearization for the IMU measurements
        double _solution_time;  // last solution time

        // main math model for the IMU state/covariance propagation
        GeneratedMatrices _state_space_model;  // from IMU matrices.hpp
};