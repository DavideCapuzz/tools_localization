#include "UKF.h"
#include <cmath>

using namespace Eigen;

static double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2. * M_PI;
    while (angle < -M_PI) angle += 2. * M_PI;
    return angle;
}

UKF::UKF(double dt, double L) : dt_(dt), L_(L) {
    n_x_ = 4;
    n_sigma_ = 2 * n_x_ + 1;
    lambda_ = 3 - n_x_;

    weights_m_ = VectorXd(n_sigma_);
    weights_c_ = VectorXd(n_sigma_);
    weights_m_(0) = lambda_ / (lambda_ + n_x_);
    weights_c_(0) = weights_m_(0);
    for (int i = 1; i < n_sigma_; ++i) {
        weights_m_(i) = weights_c_(i) = 0.5 / (n_x_ + lambda_);
    }

    Q_ = Matrix4d::Identity() * 0.1;
    R_ = Matrix2d::Identity() * 0.5;
}

void UKF::init(const Vector4d& x0, const Matrix4d& P0) {
    x_ = x0;
    P_ = P0;
}

MatrixXd UKF::generateSigmaPoints() {
    MatrixXd sigma_points(n_x_, n_sigma_);
    MatrixXd A = P_.llt().matrixL();

    sigma_points.col(0) = x_;
    for (int i = 0; i < n_x_; ++i) {
        sigma_points.col(i + 1)        = x_ + sqrt(lambda_ + n_x_) * A.col(i);
        sigma_points.col(i + 1 + n_x_) = x_ - sqrt(lambda_ + n_x_) * A.col(i);
    }
    return sigma_points;
}

Vector4d UKF::processModel(const Vector4d& x, double a, double delta) {
    double px = x(0), py = x(1), theta = x(2), v = x(3);
    Vector4d x_pred;
    x_pred(0) = px + v * cos(theta) * dt_;
    x_pred(1) = py + v * sin(theta) * dt_;
    x_pred(2) = theta + (v / L_) * tan(delta) * dt_;
    x_pred(3) = v + a * dt_;
    x_pred(2) = normalizeAngle(x_pred(2));
    return x_pred;
}

Vector2d UKF::measurementModel(const Vector4d& x) {
    Vector2d z;
    z << x(0), x(1);
    return z;
}

void UKF::predict(double a, double delta) {
    MatrixXd sigma_points = generateSigmaPoints();

    // Predict sigma points
    MatrixXd sigma_pred(n_x_, n_sigma_);
    for (int i = 0; i < n_sigma_; ++i) {
        sigma_pred.col(i) = processModel(sigma_points.col(i), a, delta);
    }

    // Predict state mean
    x_.setZero();
    for (int i = 0; i < n_sigma_; ++i)
        x_ += weights_m_(i) * sigma_pred.col(i);
    x_(2) = normalizeAngle(x_(2));

    // Predict covariance
    P_.setZero();
    for (int i = 0; i < n_sigma_; ++i) {
        Vector4d dx = sigma_pred.col(i) - x_;
        dx(2) = normalizeAngle(dx(2));
        P_ += weights_c_(i) * dx * dx.transpose();
    }
    P_ += Q_;
}

void UKF::update(const Vector2d& z) {
    MatrixXd sigma_points = generateSigmaPoints();

    // Predict sigma points through process model first
    MatrixXd sigma_pred(n_x_, n_sigma_);
    for (int i = 0; i < n_sigma_; ++i) {
        sigma_pred.col(i) = processModel(sigma_points.col(i), 0, 0);  // Use previous prediction
    }

    // Predict measurement sigma points
    MatrixXd Zsig(2, n_sigma_);
    for (int i = 0; i < n_sigma_; ++i) {
        Zsig.col(i) = measurementModel(sigma_pred.col(i));
    }

    // Predicted measurement mean
    Vector2d z_pred = Vector2d::Zero();
    for (int i = 0; i < n_sigma_; ++i)
        z_pred += weights_m_(i) * Zsig.col(i);

    // Innovation covariance S
    Matrix2d S = Matrix2d::Zero();
    for (int i = 0; i < n_sigma_; ++i) {
        Vector2d dz = Zsig.col(i) - z_pred;
        S += weights_c_(i) * dz * dz.transpose();
    }
    S += R_;

    // Cross-covariance
    Matrix<double, 4, 2> Tc = Matrix<double, 4, 2>::Zero();
    for (int i = 0; i < n_sigma_; ++i) {
        Vector4d dx = sigma_pred.col(i) - x_;
        dx(2) = normalizeAngle(dx(2));
        Vector2d dz = Zsig.col(i) - z_pred;
        Tc += weights_c_(i) * dx * dz.transpose();
    }

    // Kalman gain
    Matrix<double, 4, 2> K = Tc * S.inverse();

    // Update state and covariance
    Vector2d y = z - z_pred;
    x_ = x_ + K * y;
    x_(2) = normalizeAngle(x_(2));
    P_ = P_ - K * S * K.transpose();
}

Vector4d UKF::getState() const {
    return x_;
}

Matrix4d UKF::getCovariance() const {
    return P_;
}