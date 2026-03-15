#include "core/tracker.hpp"
#include <cstdio>

namespace baddy {

static Matrix6d build_Q(const EkfConfig& c)
{
    Matrix6d Q = Matrix6d::Zero();
    const double sp2 = c.sigma_p * c.sigma_p;
    const double sv2 = c.sigma_v * c.sigma_v;
    Q.diagonal() << sp2, sp2, sp2, sv2, sv2, sv2;
    return Q;
}

static Matrix3d build_R(const EkfConfig& c)
{
    Matrix3d R = Matrix3d::Zero();
    R.diagonal() << c.sigma_cam_x * c.sigma_cam_x,
                    c.sigma_cam_y * c.sigma_cam_y,
                    c.sigma_cam_z * c.sigma_cam_z;
    return R;
}

Tracker::Tracker(const EkfConfig& cfg)
    : cfg_(cfg),
      ekf_(cfg.alpha, cfg.gravity, build_Q(cfg), build_R(cfg)),
      predictor_(cfg.alpha, cfg.gravity, cfg.target_intercept_y, cfg.max_predict_time)
{}

void Tracker::reset()
{
    ekf_.reset();
    state_ = TrackerState::SEARCHING;
    have_first_ = false;
    coast_count_ = 0;
    last_ts_ns_ = 0;
}

TrackerResult Tracker::process_measurement(const CourtPosition& pos, int64_t timestamp_ns)
{
    const Vector3d z = pos.to_vec();

    switch (state_) {
    case TrackerState::SEARCHING: {
        if (!have_first_) {
            first_pos_ = pos;
            first_ts_ns_ = timestamp_ns;
            have_first_ = true;
            return make_result();
        }
        // Second detection: estimate velocity and initialise the EKF.
        const double dt = static_cast<double>(timestamp_ns - first_ts_ns_) * 1e-9;
        if (dt < 1e-9) {
            // Timestamps too close — treat as duplicate frame.
            return make_result();
        }
        const Vector3d vel = (z - first_pos_.to_vec()) / dt;

        Vector6d x0;
        x0 << z, vel;

        Matrix6d P0 = Matrix6d::Zero();
        P0.diagonal() << cfg_.initial_p_position, cfg_.initial_p_position, cfg_.initial_p_position,
                          cfg_.initial_p_velocity, cfg_.initial_p_velocity, cfg_.initial_p_velocity;

        ekf_.initialize(x0, P0);
        state_ = TrackerState::TRACKING;
        last_ts_ns_ = timestamp_ns;
        coast_count_ = 0;
        return make_result();
    }

    case TrackerState::TRACKING: {
        const double dt = static_cast<double>(timestamp_ns - last_ts_ns_) * 1e-9;
        if (dt > 1e-9) {
            ekf_.predict(dt);
            ekf_.update(z);
        }
        last_ts_ns_ = timestamp_ns;
        coast_count_ = 0;
        return make_result();
    }

    case TrackerState::LOST:
        // Re-acquired after loss — start fresh.
        reset();
        return process_measurement(pos, timestamp_ns);
    }

    return make_result(); // unreachable, silences compiler warning
}

TrackerResult Tracker::process_no_detection(int64_t timestamp_ns)
{
    if (state_ == TrackerState::TRACKING) {
        const double dt = static_cast<double>(timestamp_ns - last_ts_ns_) * 1e-9;
        if (dt > 1e-9)
            ekf_.predict(dt);
        last_ts_ns_ = timestamp_ns;
        ++coast_count_;

        if (coast_count_ >= cfg_.max_coast_frames) {
            state_ = TrackerState::LOST;
            std::printf("[tracker] track lost after %d coasted frames\n", coast_count_);
        }
    }
    return make_result();
}

TrackerResult Tracker::make_result() const
{
    TrackerResult r;
    r.state = state_;
    r.coast_count = coast_count_;

    if (ekf_.initialized()) {
        r.ekf_state = ekf_.state();
        r.prediction = predictor_.predict(ekf_.state());
    }
    return r;
}

} // namespace baddy
