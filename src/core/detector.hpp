#pragma once

#include "core/types.hpp"
#include "core/config.hpp"
#include <opencv2/core.hpp>

namespace baddy {

class Detector {
public:
    explicit Detector(const DetectionConfig& cfg);

    // Detect the birdie in a single rectified frame.
    Detection detect(const cv::Mat& frame) const;

    // Live-update thresholds (e.g. from trackbar callbacks).
    void set_hsv_low(int h, int s, int v);
    void set_hsv_high(int h, int s, int v);

private:
    cv::Scalar hsv_low_;
    cv::Scalar hsv_high_;
    int min_contour_area_;
    int morph_kernel_size_;
};

} // namespace baddy
