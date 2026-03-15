#pragma once

#include "core/types.hpp"
#include "core/ekf.hpp"
#include "core/landing_predictor.hpp"
#include "core/config.hpp"

namespace baddy {

struct TrackerResult {
    TrackerState     state       = TrackerState::SEARCHING;
    LandingPrediction prediction;
    Vector6d         ekf_state   = Vector6d::Zero();
    int              coast_count = 0;
};

class Tracker {
public:
    explicit Tracker(const EkfConfig& cfg);

    // Feed a court-frame measurement.  Returns the current tracker output.
    TrackerResult process_measurement(const CourtPosition& pos, int64_t timestamp_ns);

    // Signal that no detection was found this frame.
    TrackerResult process_no_detection(int64_t timestamp_ns);

    TrackerState state() const { return state_; }
    void reset();

private:
    TrackerResult make_result() const;

    EkfConfig        cfg_;
    Ekf              ekf_;
    LandingPredictor predictor_;
    TrackerState     state_ = TrackerState::SEARCHING;

    // First detection (used to bootstrap velocity estimate).
    CourtPosition first_pos_;
    int64_t       first_ts_ns_ = 0;
    bool          have_first_  = false;

    int64_t last_ts_ns_  = 0;
    int     coast_count_ = 0;
};

} // namespace baddy
