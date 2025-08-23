
#include <Eigen/Dense>

class UKF {
public:
    UKF(double dt, double L);

    void init(const Eigen::Vector4d& x0, const Eigen::Matrix4d& P0);
    void predict(double a, double delta);
    void update(const Eigen::Vector2d& z);

    Eigen::Vector4d getState() const;
    Eigen::Matrix4d getCovariance() const;

private:
    double dt_, L_;
    int n_x_;        // State dimension
    int n_sigma_;    // Number of sigma points

    double lambda_;  // Scaling parameter
    Eigen::VectorXd weights_m_;
    Eigen::VectorXd weights_c_;

    Eigen::Vector4d x_;
    Eigen::Matrix4d P_;
    Eigen::Matrix4d Q_;
    Eigen::Matrix2d R_;

    Eigen::MatrixXd generateSigmaPoints();
    Eigen::Vector4d processModel(const Eigen::Vector4d& x, double a, double delta);
    Eigen::Vector2d measurementModel(const Eigen::Vector4d& x);
};

// USAGE


// int main() {
//     UKFVehicle ukf(0.1, 2.0);

//     Eigen::Vector4d x0;
//     x0 << 0, 0, 0, 0;
//     Eigen::Matrix4d P0 = Eigen::Matrix4d::Identity();
//     ukf.init(x0, P0);

//     for (int i = 0; i < 10; ++i) {
//         double a = 1.0;
//         double delta = M_PI / 18;

//         ukf.predict(a, delta);

//         // Simulated noisy GPS
//         Eigen::Vector2d z;
//         z << ukf.getState()(0) + 0.1, ukf.getState()(1) - 0.1;

//         ukf.update(z);

//         std::cout << "UKF Step " << i << ":\n" << ukf.getState().transpose() << "\n\n";
//     }

//     return 0;
// }