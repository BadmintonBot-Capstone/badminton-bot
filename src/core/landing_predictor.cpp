#include "core/landing_predictor.hpp"
#include "core/birdie_dynamics.hpp"

namespace baddy {

LandingPredictor::LandingPredictor(double alpha, double g,
                                   double target_y, double max_time)
    : alpha_(alpha), g_(g), target_y_(target_y), max_time_(max_time) {}

LandingPrediction LandingPredictor::predict(const Vector6d& state) const
{
    // Already below target — nothing useful to predict.
    if (state[PY] <= target_y_)
        return {{state[PX], state[PY], state[PZ]}, 0.0, false};

    auto result = integrate_to_height(state, target_y_, alpha_, g_, max_time_);

    LandingPrediction lp;
    lp.valid = result.reached;
    if (result.reached) {
        lp.position = CourtPosition::from_vec(result.position);
        lp.eta_seconds = result.time;
    }
    return lp;
}

} // namespace baddy
