#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>

#include "UKF.hpp"
#include "safe_cholesky.hpp"

using json = nlohmann::json;


void UKF::read_configs(std::ifstream& inFile) {
    UKFParams hold_config;

    // access thre required members
    json j;
    inFile >> j;  // streams contents of the config file into 'j'

    // we can now access the features of the configs like a map or dict
    /// NOTE: the UKF only uses a subset of these
    hold_config.gx = j["gx"];
    hold_config.gy = j["gy"];
    hold_config.gz = j["gz"];

    // FOGMP process on biases config
    hold_config.tau_a = j["tau_ax"];
    hold_config.tau_g = j["tau_gx"];
    hold_config.sig_a = j["sig_ax"];
    hold_config.sig_g = j["sig_gx"];

    // scaling params for UKF
    _alpha = j["alpha"];
    _beta = j["beta"];
    _kappa = j["kappa"];

    // get measurement file path
    _measurement_file_path = j["path_to_measurement"];

    // example printouts
    std::cout << "Configuration loaded successfully.\n";
    std::cout << "Accelerometer time constant (tau_a): " << hold_config.tau_a << "\n";
    std::cout << "Gyro time constant (tau_g): " << hold_config.tau_g << "\n";
    _params = hold_config;  // assign configurations
}

UKF::UKF(double alpha, double beta, double kappa, UKFParams params):
    _params(params),
    _alpha(alpha),
    _beta(beta),
    _kappa(kappa) {

    // calculate scaling params
    _lambda = pow(_alpha, 2) * (N + _kappa) - N;
    _gamma = pow(N + _lambda, 0.5);

    // weights for sigma point mean and covariance
    _Wm.setConstant(0.5 / (N + _lambda));
    _Wc = _Wm; // copy

    _Wm(0) = _lambda / (N + _lambda);
    _Wc(0) = _Wm(0) + (1 - pow(_alpha, 2) + _beta);

    // TMP PROCESS NOISE
    _Q.setIdentity();
    _Q *= 1e-3;
}

UKF::UKF(const std::string& configs_path): 
    _imu_available(false), 
    _gnss_available(false), 
    _last_imu_time(0), 
    _last_gnss_time(0) 
{
    // read in configurables
    // load in the system configurations
    std::ifstream inFile(configs_path);
    if (!inFile.is_open()) {
      std::cerr << "Could not open config file: " << configs_path << std::endl;
    }

    // allocate the measurement file path and read in UKF params
    read_configs(inFile);

    // calculate scaling params
    double lambda_raw = _alpha * _alpha * (N + _kappa) - N;
    _lambda = std::max(lambda_raw, -0.95 * N);
    _gamma = std::sqrt(N + _lambda);

    // weights for sigma point mean and covariance
    _Wm.setConstant(0.5 / (N + _lambda));
    _Wc = _Wm; // copy

    _Wm(0) = _lambda / (N + _lambda);
    _Wc(0) = _Wm(0) + (1 - pow(_alpha, 2) + _beta);

    // TMP PROCESS NOISE
    _Q.setIdentity();
    _Q *= 1e-3;

    ukf_log() << "[UKF] _lambda = " << _lambda << ", _gamma = " << _gamma << "\n";

    // create the queues for holding incoming measurements
    _imu_queue = std::make_unique<ThreadQueue<ControlInput>>(1000);
    _gnss_queue = std::make_unique<ThreadQueue<Observable>>(1000);
}

void UKF::start_filter() {}

void UKF::read_imu(ImuData imu_measurement) {
    ControlInput imu_reading = imu_measurement.matrix_form_measurement;

    // signal to the filter that we're ready to process IMU data
    _imu_available = true;
    _imu_queue->push(imu_reading);

    // of we have a GNSS measurement available, we can start an update

    double dt = abs(_solution_time - imu_measurement.measurement_time);
    predict(imu_reading, dt);

    _solution_time = imu_measurement.measurement_time;  // update solution time
}

void UKF::read_gps(Observable observable_measurement) {
    // signal to the filter that we're ready to process GNSS data
    _gnss_available = true;
    _gnss_queue->push(observable_measurement);

    // update(observable_measurement.observation, observable_measurement.R);

    _solution_time = observable_measurement.timestamp;
}

const StateVec& UKF::get_state() const{
    return _x;
}

const CovMat& UKF::get_covariance() const{
    return _P;
}

void UKF::initialize(const StateVec& initial_state, const CovMat& initial_covariance) {
    _x = initial_state;
    _P = initial_covariance;
    _solution_time = 0;
}

void UKF::generate_sigma_points(
    const StateVec& mu,
    const CovMat& P,
    SigmaPointArray& sigma_points) {

    // tmp logs
    ukf_log() << "Initial covariance diag: " << P.diagonal().transpose() << "\n";
    ukf_log() << "Initial state mu: " << mu.transpose() << "\n";

    // scale the covariance matrix
    CovMat scaled_P = (N + _lambda) * P;

    // ensure the matrix is symmetric and positive definite
    // the function will also perform the cholesky decomp after these steps are taken
    CovMat S;
    bool success = safe_cholesky(scaled_P, S);

    if (!success) {
        std::cerr << "[UKF] ERROR: Failed to compute Cholesky decomposition despite stabilization attempts.\n";
        std::cerr << "[UKF] Matrix was:\n" << scaled_P << "\n";
        throw std::runtime_error("UKF: Cholesky decomposition failed");
    }

    // assign first sigma point
    sigma_points[0] = mu;

    // generate symmetric sigma points
    for (uint8_t i = 0; i < N; ++i) {
        sigma_points[i + 1]       = mu + _gamma * S.col(i);
        sigma_points[N + i + 1]   = mu - _gamma * S.col(i);
    }

    ukf_log() << "[UKF] Raw P:\n" << P << "\n";
    ukf_log() << "[UKF] Scaled P:\n" << scaled_P << "\n";

}

void UKF::predict(
    const ControlInput& u, double dt) {

    // use the nomlinear dynamics to propagate the sigma points into predicted states
    SigmaPointArray sigma_points;
    generate_sigma_points(_x, _P, sigma_points); // get new sigma points

    // propagate each sigma point through the nonlinear dynamics using rk4
    SigmaPointArray propagated_sigmas;
    for (uint8_t i = 0; i < NumSigma; ++i) {
        propagated_sigmas[i] = rk4_step(sigma_points[i], u, dt, _params);
    }

    // compute predicted mean and covariance
    StateVec mu_pred;
    mu_pred.setZero();
    for (uint8_t i = 0; i < NumSigma; ++i) {
        mu_pred += _Wm[i] * propagated_sigmas[i];
    }

    CovMat P_pred;
    P_pred = _Q; // covariance update is of the form var(x) + Q
    for (uint8_t i = 0; i < NumSigma; ++i) {
        // "measure" the covariance based on deviations in the sigma points from the mean
        StateVec dx = propagated_sigmas[i] - mu_pred;
        P_pred += _Wc[i] * (dx * dx.transpose());
    }

    // update state & covariance
    _x = mu_pred;
    _P = P_pred;
    _sigma_points = propagated_sigmas;
}

void UKF::update(const MeasVec& z, const MeasCov& R) {
    // propagate through measurement model
    std::array<MeasVec, NumSigma> z_sigma;
    for (uint8_t i = 0; i < NumSigma; ++i) {
        z_sigma[i] = _sigma_points[i].head<Z>(); // observing position + velocity
    }

    // predicted measurement mean
    MeasVec z_pred = MeasVec::Zero();
    for (uint8_t i = 0; i < NumSigma; ++i) {
        z_pred += _Wm[i] * z_sigma[i];
    }

    // log resudial in this prediction
    Eigen::Matrix<double, Z, 1> residual = z - z_pred;

    // innovation covariance and cross-covariance
    MeasCov S = R;
    Eigen::Matrix<double, N, Z> P_xz = Eigen::Matrix<double, N, Z>::Zero();
    for (uint8_t i = 0; i < NumSigma; ++i) {
        MeasVec dz = z_sigma[i] - z_pred;
        StateVec dx = _sigma_points[i] - _x;
        S += _Wc[i] * dz * dz.transpose();
        P_xz += _Wc[i] * dx * dz.transpose();
    }

    // Kalman gain
    Eigen::Matrix<double, N, Z> K = P_xz * S.inverse();

    // state update
    _x += K * (z - z_pred);
    _P -= K * S * K.transpose();
}