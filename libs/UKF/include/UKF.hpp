#pragma once

#include <cstdint>
#include <Eigen/Dense>
#include <string>

#include "estimator_interface.hpp"
#include "ukf_defs.hpp"
#include "IMU_Matrices.hpp"
#include "thread_safe_queue.hpp"

// using N and M for adaptive sizing depending on model without heap allocations
class UKF : public Estimator {
public:
    /// NOTE: use of an 8 bit integer limits the number of sigma points to 255
    static constexpr uint8_t NumSigma = 2 * N + 1; // minimum sample from UKF theory

    UKF(double alpha, double beta, double kappa, UKFParams params);

    // alternate constructor for sim mode
    UKF(const std::string& configs_path);
    UKF();

    void init(const std::string& configs_path);

    // sets params for system and gets measurement file path
    void read_configs(std::ifstream& inFile);

    // kicks off processing loop
    void start_filter();

    // kickoff for sim mode processing
    void read_imu(ImuData imu_measurement);
    void read_gps(Observable observable_measurement);

    /** inherited functions from the estimator class **/
    void initialize(const StateVec& initial_state, const CovMat& initial_covariance);

    void predict(const ControlInput& u, double dt);

    void update(const MeasVec& z, const MeasCov& R);

    // getters for state and covariance
    const StateVec& get_state() const;
    const CovMat& get_covariance() const;

private:
    // config
    UKFParams _params;

    // UKF scaling parameters
    double _alpha, _beta, _kappa, _lambda, _gamma;

    // weight params
    Eigen::Matrix<double, NumSigma, 1> _Wm;
    Eigen::Matrix<double, NumSigma, 1> _Wc;

    // state variables
    CovMat _P;
    StateVec _x;

    // process noise matrix
    CovMat _Q;

    // current sigma points
    SigmaPointArray _sigma_points;

    std::string _measurement_file_path;
    std::string _ground_truth_path;
    double _solution_time;  // last solution time

    // queues for holding incoming measurements
    std::unique_ptr<ThreadQueue<ControlInput>> _imu_queue;
    std::unique_ptr<ThreadQueue<Observable>> _gnss_queue;

    // trackers for syncing imu and GNSS solutions'
    bool _imu_available;
    bool _gnss_available;
    double _last_imu_time;
    double _last_gnss_time;

    void generate_sigma_points(const StateVec& mu, const CovMat& P, SigmaPointArray& sigma_points);
};