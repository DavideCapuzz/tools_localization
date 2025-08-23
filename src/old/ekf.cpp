#include "EKF.h"
#include <cmath>

using namespace Eigen;

static double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}

EKF::EKF(double dt, double L) : dt_(dt), L_(L) {
    Q_ = Matrix4d::Identity() * 0.1;
    R_ = Matrix2d::Identity() * 0.5;
}

void EKF::init(const Vector4d& x0, const Matrix4d& P0) {
    x_ = x0;
    P_ = P0;
}

void EKF::predict(double a, double delta) {
    double theta = x_(2);
    double v = x_(3);

    // Predict state
    Vector4d x_pred;
    x_pred(0) = x_(0) + v * cos(theta) * dt_;
    x_pred(1) = x_(1) + v * sin(theta) * dt_;
    x_pred(2) = x_(2) + (v / L_) * tan(delta) * dt_;
    x_pred(3) = x_(3) + a * dt_;
    x_pred(2) = normalizeAngle(x_pred(2));

    // Jacobian F
    Matrix4d F = Matrix4d::Identity();
    F(0, 2) = -v * sin(theta) * dt_;
    F(0, 3) = cos(theta) * dt_;
    F(1, 2) = v * cos(theta) * dt_;
    F(1, 3) = sin(theta) * dt_;
    F(2, 3) = (1.0 / L_) * tan(delta) * dt_;

    // Predict covariance
    P_ = F * P_ * F.transpose() + Q_;
    x_ = x_pred;
}

void EKF::update(const Vector2d& z) {
    // Measurement model: h(x) = [x, y]
    Vector2d z_pred;
    z_pred << x_(0), x_(1);

    Matrix<double, 2, 4> H = Matrix<double, 2, 4>::Zero();
    H(0, 0) = 1;
    H(1, 1) = 1;

    Vector2d y = z - z_pred;
    Matrix2d S = H * P_ * H.transpose() + R_;
    Matrix<double, 4, 2> K = P_ * H.transpose() * S.inverse();

    x_ = x_ + K * y;
    Matrix4d I = Matrix4d::Identity();
    P_ = (I - K * H) * P_;
}

Vector4d EKF::getState() const {
    return x_;
}

Matrix4d EKF::getCovariance() const {
    return P_;
}
