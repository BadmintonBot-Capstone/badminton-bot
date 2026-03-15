#include "core/ekf.hpp"
#include "core/birdie_dynamics.hpp"

namespace baddy {

// H is constant: we observe [px, py, pz] directly.
const Matrix36d Ekf::H_ = (Matrix36d() <<
    1, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0
).finished();

Ekf::Ekf(double alpha, double g, const Matrix6d& Q, const Matrix3d& R)
    : alpha_(alpha), g_(g), Q_(Q), R_(R)
{
    x_.setZero();
    P_.setIdentity();
}

void Ekf::initialize(const Vector6d& state, const Matrix6d& P)
{
    x_ = state;
    P_ = P;
    initialized_ = true;
}

void Ekf::predict(double dt)
{
    const Matrix6d F = compute_jacobian(x_, dt, alpha_, g_);
    x_ = predict_state(x_, dt, alpha_, g_);
    P_ = F * P_ * F.transpose() + Q_;
}

void Ekf::update(const Vector3d& measurement)
{
    // Innovation.
    const Vector3d y = measurement - H_ * x_;

    // Innovation covariance.
    const Matrix3d S = H_ * P_ * H_.transpose() + R_;

    // Kalman gain.
    const Eigen::Matrix<double, 6, 3> K = P_ * H_.transpose() * S.inverse();

    // State update.
    x_ = x_ + K * y;

    // Covariance update (Joseph form for numerical stability).
    const Matrix6d I_KH = Matrix6d::Identity() - K * H_;
    P_ = I_KH * P_ * I_KH.transpose() + K * R_ * K.transpose();
}

void Ekf::reset()
{
    x_.setZero();
    P_.setIdentity();
    initialized_ = false;
}

} // namespace baddy
