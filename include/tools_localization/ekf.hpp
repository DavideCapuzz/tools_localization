#include <Eigen/Dense>

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class EKF {
public:
    EKF(double dt, double L);

    void init(const Eigen::Vector4d& x0, const Eigen::Matrix4d& P0);
    void predict(double a, double delta);
    void update(const Eigen::Vector2d& z);

    Eigen::Vector4d getState() const;
    Eigen::Matrix4d getCovariance() const;

private:
    double dt_;     // Time step
    double L_;      // Wheelbase

    Eigen::Vector4d x_;       // State: [x, y, theta, v]
    Eigen::Matrix4d P_;       // Covariance
    Eigen::Matrix4d Q_;       // Process noise
    Eigen::Matrix2d R_;       // Measurement noise
};

// USAGE


// int main() {
//     double dt = 0.1;
//     double L = 2.0;

//     EKFVehicle ekf(dt, L);

//     Eigen::Vector4d x0;
//     x0 << 0, 0, 0, 0;

//     Eigen::Matrix4d P0 = Eigen::Matrix4d::Identity() * 1.0;
//     ekf.init(x0, P0);

//     for (int i = 0; i < 10; ++i) {
//         double a = 1.0;                   // Acceleration
//         double delta = M_PI / 18;        // 10 degrees

//         ekf.predict(a, delta);

//         // Simulated noisy GPS measurement
//         Eigen::Vector2d z;
//         z << ekf.getState()(0) + 0.1, ekf.getState()(1) - 0.1;
//         ekf.update(z);

//         std::cout << "Step " << i << " State:\n" << ekf.getState().transpose() << "\n\n";
//     }

//     return 0;
// }