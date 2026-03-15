#include "core/detector.hpp"
#include <opencv2/imgproc.hpp>

namespace baddy {

Detector::Detector(const DetectionConfig& cfg)
    : hsv_low_(cfg.hsv_low[0], cfg.hsv_low[1], cfg.hsv_low[2]),
      hsv_high_(cfg.hsv_high[0], cfg.hsv_high[1], cfg.hsv_high[2]),
      min_contour_area_(cfg.min_contour_area),
      morph_kernel_size_(cfg.morph_kernel_size)
{}

Detection Detector::detect(const cv::Mat& frame) const
{
    Detection det;

    cv::Mat hsv, mask;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, hsv_low_, hsv_high_, mask);

    cv::Mat kernel = cv::getStructuringElement(
        cv::MORPH_ELLIPSE, {morph_kernel_size_, morph_kernel_size_});
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN,  kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty()) return det;

    // Find the largest contour.
    int best = 0;
    double best_area = cv::contourArea(contours[0]);
    for (int i = 1; i < static_cast<int>(contours.size()); ++i) {
        double a = cv::contourArea(contours[i]);
        if (a > best_area) { best_area = a; best = i; }
    }

    if (best_area < min_contour_area_) return det;

    cv::Moments m = cv::moments(contours[best]);
    if (m.m00 < 1e-6) return det;

    det.found   = true;
    det.pixel_x = m.m10 / m.m00;
    det.pixel_y = m.m01 / m.m00;
    det.area    = best_area;
    return det;
}

void Detector::set_hsv_low(int h, int s, int v)  { hsv_low_  = {double(h), double(s), double(v)}; }
void Detector::set_hsv_high(int h, int s, int v) { hsv_high_ = {double(h), double(s), double(v)}; }

} // namespace baddy
