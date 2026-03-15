#pragma once

#include "core/types.hpp"
#include "core/config.hpp"
#include <opencv2/core.hpp>

namespace baddy {

class StereoRig {
public:
    // Initialise from calibration data.  Precomputes rectification maps.
    explicit StereoRig(const CalibrationData& cal, int width, int height);

    // Rectify a synchronised frame pair in place.
    void rectify(const cv::Mat& raw_left,  const cv::Mat& raw_right,
                 cv::Mat& rect_left, cv::Mat& rect_right) const;

    // Triangulate from pixel centroids in the rectified images.
    // Returns the point in camera frame (not court frame).
    Vector3d triangulate(double px_left_x, double px_left_y,
                         double px_right_x, double px_right_y) const;

    // Transform a camera-frame point to court coordinates.
    CourtPosition to_court(const Vector3d& camera_xyz) const;

private:
    cv::Mat map_l1_, map_l2_, map_r1_, map_r2_;
    cv::Mat P1_, P2_;
    Eigen::Matrix4d T_camera_to_court_;
};

} // namespace baddy
