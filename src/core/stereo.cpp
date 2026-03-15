#include "core/stereo.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace baddy {

StereoRig::StereoRig(const CalibrationData& cal, int width, int height)
    : T_camera_to_court_(cal.T_camera_to_court)
{
    P1_ = cal.P1.clone();
    P2_ = cal.P2.clone();

    cv::initUndistortRectifyMap(cal.M1, cal.D1, cal.R1, cal.P1,
                                {width, height}, CV_16SC2, map_l1_, map_l2_);
    cv::initUndistortRectifyMap(cal.M2, cal.D2, cal.R2, cal.P2,
                                {width, height}, CV_16SC2, map_r1_, map_r2_);
}

void StereoRig::rectify(const cv::Mat& raw_left,  const cv::Mat& raw_right,
                         cv::Mat& rect_left, cv::Mat& rect_right) const
{
    cv::remap(raw_left,  rect_left,  map_l1_, map_l2_, cv::INTER_LINEAR);
    cv::remap(raw_right, rect_right, map_r1_, map_r2_, cv::INTER_LINEAR);
}

Vector3d StereoRig::triangulate(double px_left_x, double px_left_y,
                                double px_right_x, double px_right_y) const
{
    cv::Mat pt_l = (cv::Mat_<double>(2, 1) << px_left_x, px_left_y);
    cv::Mat pt_r = (cv::Mat_<double>(2, 1) << px_right_x, px_right_y);

    cv::Mat pts_4d;
    cv::triangulatePoints(P1_, P2_, pt_l, pt_r, pts_4d);

    // Convert from homogeneous.
    const double w = pts_4d.at<double>(3, 0);
    return {pts_4d.at<double>(0, 0) / w,
            pts_4d.at<double>(1, 0) / w,
            pts_4d.at<double>(2, 0) / w};
}

CourtPosition StereoRig::to_court(const Vector3d& camera_xyz) const
{
    Eigen::Vector4d h;
    h << camera_xyz, 1.0;
    Eigen::Vector4d court = T_camera_to_court_ * h;
    return {court.x(), court.y(), court.z()};
}

} // namespace baddy
