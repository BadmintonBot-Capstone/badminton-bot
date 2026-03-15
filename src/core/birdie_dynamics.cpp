#include "core/birdie_dynamics.hpp"
#include <cmath>

namespace baddy {

Vector6d compute_derivatives(const Vector6d& state, double alpha, double g)
{
    const double vx = state[VX];
    const double vy = state[VY];
    const double vz = state[VZ];
    const double speed = std::sqrt(vx * vx + vy * vy + vz * vz);

    Vector6d d;
    d[PX] = vx;
    d[PY] = vy;
    d[PZ] = vz;
    d[VX] = -alpha * speed * vx;
    d[VY] = -g - alpha * speed * vy;
    d[VZ] = -alpha * speed * vz;
    return d;
}

Vector6d predict_state(const Vector6d& state, double dt, double alpha, double g)
{
    const double vx = state[VX];
    const double vy = state[VY];
    const double vz = state[VZ];
    const double speed = std::sqrt(vx * vx + vy * vy + vz * vz);

    const double ax = -alpha * speed * vx;
    const double ay = -g - alpha * speed * vy;
    const double az = -alpha * speed * vz;

    Vector6d pred;
    pred[PX] = state[PX] + vx * dt + 0.5 * ax * dt * dt;
    pred[PY] = state[PY] + vy * dt + 0.5 * ay * dt * dt;
    pred[PZ] = state[PZ] + vz * dt + 0.5 * az * dt * dt;
    pred[VX] = vx + ax * dt;
    pred[VY] = vy + ay * dt;
    pred[VZ] = vz + az * dt;
    return pred;
}

Matrix6d compute_jacobian(const Vector6d& state, double dt, double alpha, double g)
{
    const double vx = state[VX];
    const double vy = state[VY];
    const double vz = state[VZ];
    const double speed = std::sqrt(vx * vx + vy * vy + vz * vz);

    Matrix6d F = Matrix6d::Identity();

    // When speed ≈ 0 the drag partials vanish and F stays as identity.
    constexpr double kEps = 1e-12;
    if (speed < kEps)
        return F;

    const double inv_speed = 1.0 / speed;
    const double dt2 = dt * dt;

    // Precomputed products used across multiple entries.
    const double a_dt  = alpha * dt;
    const double a_dt2 = alpha * dt2 * 0.5;

    // ── Upper-right 3×3 block: ∂f_p / ∂v ─────────────────────────
    // F(PX, VX)
    F(PX, VX) = dt - a_dt2 * (vx * vx * inv_speed + speed);
    F(PX, VY) =    - a_dt2 *  vx * vy * inv_speed;
    F(PX, VZ) =    - a_dt2 *  vx * vz * inv_speed;

    F(PY, VX) =    - a_dt2 *  vx * vy * inv_speed;
    F(PY, VY) = dt - a_dt2 * (vy * vy * inv_speed + speed);
    F(PY, VZ) =    - a_dt2 *  vy * vz * inv_speed;

    F(PZ, VX) =    - a_dt2 *  vx * vz * inv_speed;
    F(PZ, VY) =    - a_dt2 *  vy * vz * inv_speed;
    F(PZ, VZ) = dt - a_dt2 * (vz * vz * inv_speed + speed);

    // ── Lower-right 3×3 block: ∂f_v / ∂v ─────────────────────────
    F(VX, VX) = 1.0 - a_dt * (vx * vx * inv_speed + speed);
    F(VX, VY) =     - a_dt *  vx * vy * inv_speed;
    F(VX, VZ) =     - a_dt *  vx * vz * inv_speed;

    F(VY, VX) =     - a_dt *  vx * vy * inv_speed;
    F(VY, VY) = 1.0 - a_dt * (vy * vy * inv_speed + speed);
    F(VY, VZ) =     - a_dt *  vy * vz * inv_speed;

    F(VZ, VX) =     - a_dt *  vx * vz * inv_speed;
    F(VZ, VY) =     - a_dt *  vy * vz * inv_speed;
    F(VZ, VZ) = 1.0 - a_dt * (vz * vz * inv_speed + speed);

    return F;
}

// ── RK4 helper for integrate_to_height ────────────────────────────
static Vector6d rk4_step(const Vector6d& state, double h, double alpha, double g)
{
    const Vector6d k1 = compute_derivatives(state,                  alpha, g);
    const Vector6d k2 = compute_derivatives(state + 0.5 * h * k1,  alpha, g);
    const Vector6d k3 = compute_derivatives(state + 0.5 * h * k2,  alpha, g);
    const Vector6d k4 = compute_derivatives(state + h * k3,        alpha, g);
    return state + (h / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}

IntegrationResult integrate_to_height(const Vector6d& state, double target_y,
                                      double alpha, double g, double max_time)
{
    constexpr double kStep = 0.001; // 1 ms fixed step

    Vector6d s = state;
    double t = 0.0;

    while (t < max_time) {
        const double h = std::min(kStep, max_time - t);
        Vector6d s_next = rk4_step(s, h, alpha, g);
        t += h;

        // Check if the birdie crossed below target_y during this step.
        if (s_next[PY] <= target_y) {
            // Linear interpolation for a tighter landing time estimate.
            const double dy = s[PY] - s_next[PY];
            const double frac = (dy > 1e-12) ? (s[PY] - target_y) / dy : 1.0;
            const double t_land = t - h + frac * h;
            const Vector3d pos = s.head<3>() + frac * (s_next.head<3>() - s.head<3>());
            return {pos, t_land, true};
        }
        s = s_next;
    }

    return {s.head<3>(), -1.0, false};
}

} // namespace baddy
