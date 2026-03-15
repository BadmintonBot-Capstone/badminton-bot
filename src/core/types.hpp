#pragma once

#include <Eigen/Dense>
#include <cstdint>

namespace baddy {

using Vector3d  = Eigen::Vector3d;
using Vector6d  = Eigen::Matrix<double, 6, 1>;
using Matrix3d  = Eigen::Matrix3d;
using Matrix6d  = Eigen::Matrix<double, 6, 6>;
using Matrix36d = Eigen::Matrix<double, 3, 6>;

// Indices into the 6-element state vector.
enum StateIdx : int { PX = 0, PY = 1, PZ = 2, VX = 3, VY = 4, VZ = 5 };

struct Detection {
    bool   found    = false;
    double pixel_x  = 0.0;
    double pixel_y  = 0.0;
    double area     = 0.0;
};

struct StereoDetection {
    Detection left;
    Detection right;
    int64_t   timestamp_ns = 0;
};

struct CourtPosition {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    Vector3d to_vec() const { return {x, y, z}; }
    static CourtPosition from_vec(const Vector3d& v) { return {v.x(), v.y(), v.z()}; }
};

struct LandingPrediction {
    CourtPosition position;
    double        eta_seconds = 0.0;
    bool          valid       = false;
};

enum class TrackerState { SEARCHING, TRACKING, LOST };

struct FrameTiming {
    double acquire_us     = 0.0;
    double rectify_us     = 0.0;
    double detect_us      = 0.0;
    double triangulate_us = 0.0;
    double ekf_us         = 0.0;
    double predict_us     = 0.0;
    double total_us       = 0.0;
};

} // namespace baddy
