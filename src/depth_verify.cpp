// depth_verify.cpp — Live HSV tuning + stereo depth verification.
// Shows detection contours, HSV masks, epipolar lines, triangulated 3D position.
// C++ port of old/depth_verify.py, using StereoCamera for config from camera.yaml.
//
// Usage:
//   ./depth_verify [--config config/] [--calibration config/calibration.yaml]
//
// Keys: 'q' quit, 'p' print current HSV values

#include "core/camera.hpp"
#include "core/config.hpp"
#include "core/stereo.hpp"
#include "core/detector.hpp"
#include "core/timing.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <cstdio>
#include <cstring>

static int h_lo, s_lo, v_lo;
static int h_hi, s_hi, v_hi;

static constexpr int DISP_W = 640;
static constexpr int DISP_H = 400;

int main(int argc, char** argv)
{
    std::string config_dir = "config";
    std::string cal_path   = "config/calibration.yaml";

    for (int i = 1; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--config") && i + 1 < argc)
            config_dir = argv[++i];
        else if (!std::strcmp(argv[i], "--calibration") && i + 1 < argc)
            cal_path = argv[++i];
    }

    auto cfg = baddy::load_config(config_dir);
    baddy::load_calibration(cfg, cal_path);

    baddy::StereoCamera camera(cfg.camera);
    baddy::StereoRig    rig(cfg.calibration, cfg.camera.width, cfg.camera.height);
    baddy::Detector     detector(cfg.detection);

    // Initialise slider values from detection config.
    h_lo = cfg.detection.hsv_low[0];  s_lo = cfg.detection.hsv_low[1];  v_lo = cfg.detection.hsv_low[2];
    h_hi = cfg.detection.hsv_high[0]; s_hi = cfg.detection.hsv_high[1]; v_hi = cfg.detection.hsv_high[2];

    cv::namedWindow("Controls", cv::WINDOW_AUTOSIZE);
    cv::createTrackbar("H lo", "Controls", &h_lo, 179);
    cv::createTrackbar("S lo", "Controls", &s_lo, 255);
    cv::createTrackbar("V lo", "Controls", &v_lo, 255);
    cv::createTrackbar("H hi", "Controls", &h_hi, 179);
    cv::createTrackbar("S hi", "Controls", &s_hi, 255);
    cv::createTrackbar("V hi", "Controls", &v_hi, 255);

    cv::namedWindow("Depth Verify", cv::WINDOW_NORMAL);
    cv::resizeWindow("Depth Verify", DISP_W * 2, DISP_H * 2);

    camera.start();
    std::puts("[depth_verify] running. 'q' quit, 'p' print HSV values.\n");

    cv::Mat left, right, rect_l, rect_r;
    int64_t ts_ns = 0;
    int frame_id = 0;

    while (true) {
        if (!camera.grab(left, right, ts_ns))
            continue;

        // Update detector from sliders.
        detector.set_hsv_low(h_lo, s_lo, v_lo);
        detector.set_hsv_high(h_hi, s_hi, v_hi);

        // ── Rectify ──
        double rectify_us = 0;
        {
            baddy::ScopedTimer t(&rectify_us);
            rig.rectify(left, right, rect_l, rect_r);
        }

        // ── Detect ──
        baddy::Detection det_l, det_r;
        double detect_us = 0;
        {
            baddy::ScopedTimer t(&detect_us);
            det_l = detector.detect(rect_l);
            det_r = detector.detect(rect_r);
        }

        // ── HSV masks (both cameras) ──
        cv::Mat hsv_l, hsv_r, mask_l, mask_r;
        cv::cvtColor(rect_l, hsv_l, cv::COLOR_BGR2HSV);
        cv::cvtColor(rect_r, hsv_r, cv::COLOR_BGR2HSV);
        cv::Scalar lo(h_lo, s_lo, v_lo), hi(h_hi, s_hi, v_hi);
        cv::inRange(hsv_l, lo, hi, mask_l);
        cv::inRange(hsv_r, lo, hi, mask_r);

        // ── Triangulate ──
        double tri_us = 0;
        char depth_text[256] = "Depth: ---";
        if (det_l.found && det_r.found) {
            baddy::ScopedTimer t(&tri_us);
            auto cam_xyz = rig.triangulate(det_l.pixel_x, det_l.pixel_y,
                                           det_r.pixel_x, det_r.pixel_y);
            auto court = rig.to_court(cam_xyz);
            double disparity = det_l.pixel_x - det_r.pixel_x;

            std::snprintf(depth_text, sizeof(depth_text),
                          "[%.3f, %.3f, %.3f] m  disp=%.1f px",
                          court.x, court.y, court.z, disparity);

            if (frame_id % 10 == 0) {
                std::printf("[depth] xyz=[%.3f, %.3f, %.3f] m  disp=%.1f px  "
                            "rect=%.0fus det=%.0fus tri=%.0fus\n",
                            court.x, court.y, court.z, disparity,
                            rectify_us, detect_us, tri_us);
            }
        }

        // ── Build display ──
        const double sx = double(DISP_W) / rect_l.cols;
        const double sy = double(DISP_H) / rect_l.rows;

        cv::Mat disp_l, disp_r, disp_mask_l, disp_mask_r;
        cv::resize(rect_l, disp_l, {DISP_W, DISP_H});
        cv::resize(rect_r, disp_r, {DISP_W, DISP_H});
        cv::resize(mask_l, disp_mask_l, {DISP_W, DISP_H});
        cv::resize(mask_r, disp_mask_r, {DISP_W, DISP_H});

        // Draw detection circles + epipolar lines.
        if (det_l.found) {
            int dx = int(det_l.pixel_x * sx), dy = int(det_l.pixel_y * sy);
            cv::circle(disp_l, {dx, dy}, 8, {0, 255, 0}, 2);
            cv::line(disp_l, {0, dy}, {DISP_W, dy}, {255, 0, 0}, 1);
        }
        if (det_r.found) {
            int dx = int(det_r.pixel_x * sx), dy = int(det_r.pixel_y * sy);
            cv::circle(disp_r, {dx, dy}, 8, {0, 255, 0}, 2);
            cv::line(disp_r, {0, dy}, {DISP_W, dy}, {255, 0, 0}, 1);
        }

        // Top row: cameras. Bottom row: masks.
        cv::Mat cam_row, mask_row_l, mask_row_r, mask_row, combined;
        cv::hconcat(disp_l, disp_r, cam_row);
        cv::cvtColor(disp_mask_l, mask_row_l, cv::COLOR_GRAY2BGR);
        cv::cvtColor(disp_mask_r, mask_row_r, cv::COLOR_GRAY2BGR);
        cv::hconcat(mask_row_l, mask_row_r, mask_row);

        // Depth readout on camera row.
        cv::Scalar text_color = (det_l.found && det_r.found) ?
            cv::Scalar(0, 255, 255) : cv::Scalar(0, 0, 255);
        cv::putText(cam_row, depth_text, {10, 30},
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, text_color, 2);

        // HSV bounds on mask row.
        char hsv_text[128];
        std::snprintf(hsv_text, sizeof(hsv_text),
                      "HSV: [%d,%d,%d] - [%d,%d,%d]",
                      h_lo, s_lo, v_lo, h_hi, s_hi, v_hi);
        cv::putText(mask_row, hsv_text, {10, 30},
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, {0, 255, 0}, 2);

        cv::vconcat(cam_row, mask_row, combined);
        cv::imshow("Depth Verify", combined);

        int key = cv::waitKey(1) & 0xFF;
        if (key == 'q') break;
        if (key == 'p') {
            std::printf("\nHSV low:  [%d, %d, %d]\nHSV high: [%d, %d, %d]\n\n",
                        h_lo, s_lo, v_lo, h_hi, s_hi, v_hi);
        }

        ++frame_id;
    }

    camera.stop();
    cv::destroyAllWindows();
    return 0;
}
