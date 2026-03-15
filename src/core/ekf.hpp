#pragma once

#include "core/types.hpp"

namespace baddy {

class Ekf {
public:
    Ekf(double alpha, double g,
        const Matrix6d& Q, const Matrix3d& R);

    // Set the initial state and covariance.
    void initialize(const Vector6d& state, const Matrix6d& P);

    // Predict step: propagate state and covariance by dt seconds.
    void predict(double dt);

    // Update step: fuse a position-only measurement [x, y, z].
    void update(const Vector3d& measurement);

    const Vector6d& state()      const { return x_; }
    const Matrix6d& covariance() const { return P_; }
    bool  initialized()          const { return initialized_; }

    void reset();

private:
    double alpha_;
    double g_;
    Matrix6d Q_;   // process noise covariance
    Matrix3d R_;   // measurement noise covariance

    Vector6d x_;   // state estimate
    Matrix6d P_;   // estimate covariance
    bool initialized_ = false;

    // Measurement Jacobian — constant: H = [I₃ | 0₃]
    static const Matrix36d H_;
};

} // namespace baddy
