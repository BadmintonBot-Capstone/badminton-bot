#pragma once

#include "core/types.hpp"

namespace baddy {

class LandingPredictor {
public:
    LandingPredictor(double alpha, double g,
                     double target_y, double max_time);

    // Forward-simulate from the current EKF state and return the
    // predicted landing position + time-to-arrival.
    LandingPrediction predict(const Vector6d& state) const;

    void set_target_y(double y)   { target_y_ = y; }
    void set_max_time(double t)   { max_time_ = t; }

private:
    double alpha_;
    double g_;
    double target_y_;
    double max_time_;
};

} // namespace baddy
