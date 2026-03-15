#pragma once

#include "core/types.hpp"

namespace baddy {

// Compute the time-derivative of the state: [dx, dy, dz, dvx, dvy, dvz].
// Implements the 6-ODE shuttlecock model from Shen et al.
Vector6d compute_derivatives(const Vector6d& state, double alpha, double g);

// One-step state prediction (2nd-order Taylor: p += v*dt + 0.5*a*dt²).
// Used by the EKF predict step.
Vector6d predict_state(const Vector6d& state, double dt, double alpha, double g);

// Analytic 6×6 state-transition Jacobian F = ∂f/∂x evaluated at (state, dt).
// Derived from the symbolic expressions in prototyping.ipynb.
Matrix6d compute_jacobian(const Vector6d& state, double dt, double alpha, double g);

// Forward-integrate the ODE with RK4 until the birdie descends to target_y.
// Returns {landing_position, time_to_arrival}.  If the birdie never reaches
// target_y within max_time, the returned pair has time < 0.
struct IntegrationResult {
    Vector3d position;
    double   time;
    bool     reached;
};
IntegrationResult integrate_to_height(const Vector6d& state, double target_y,
                                      double alpha, double g, double max_time);

} // namespace baddy
