// diff_detect.cpp — Frame differencing detection prototype.
// Subtracts consecutive grayscale frames to detect motion.
// No rectification, no triangulation — pure detection test.
//
// Usage:
//   ./diff_detect [--config config/]
//
// Keys: 'q' quit, 'h' toggle display on/off

#include "core/camera.hpp"
#include "core/config.hpp"
#include "core/timing.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <cstdio>
#include <cstring>
#include <vector>

static int diff_lo  = 30;
static int diff_hi  = 255;
static int min_area = 5;
static int max_area = 5000;

static constexpr int DISP_W = 640;
static constexpr int DISP_H = 400;

struct Detection {
    bool found = false;
    double pixel_x = 0, pixel_y = 0;
};

// Run frame-diff detection on a single camera's grayscale frames.
static Detection detect_diff(const cv::Mat& prev_gray, const cv::Mat& curr_gray,
                             cv::Mat& diff_out, cv::Mat& thresh_out)
{
    Detection det;
    cv::absdiff(prev_gray, curr_gray, diff_out);
    cv::inRange(diff_out, cv::Scalar(diff_lo), cv::Scalar(diff_hi), thresh_out);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(thresh_out, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    double best_area = 0;
    int best_idx = -1;
    for (int i = 0; i < static_cast<int>(contours.size()); ++i) {
        double a = cv::contourArea(contours[i]);
        if (a >= min_area && a <= max_area && a > best_area) {
            best_area = a;
            best_idx = i;
        }
    }

    if (best_idx >= 0) {
        auto m = cv::moments(contours[best_idx]);
        if (m.m00 > 0) {
            det.found = true;
            det.pixel_x = m.m10 / m.m00;
            det.pixel_y = m.m01 / m.m00;
        }
    }
    return det;
}

int main(int argc, char** argv)
{
    std::string config_dir = "config";

    for (int i = 1; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--config") && i + 1 < argc)
            config_dir = argv[++i];
    }

    auto cfg = baddy::load_config(config_dir);
    baddy::StereoCamera camera(cfg.camera);

    cv::namedWindow("Controls", cv::WINDOW_AUTOSIZE);
    cv::createTrackbar("Diff lo",  "Controls", &diff_lo, 255);
    cv::createTrackbar("Diff hi",  "Controls", &diff_hi, 255);
    cv::createTrackbar("Min area", "Controls", &min_area, 1000);
    cv::createTrackbar("Max area", "Controls", &max_area, 10000);

    cv::namedWindow("Diff Detect", cv::WINDOW_NORMAL);
    cv::resizeWindow("Diff Detect", DISP_W * 2, DISP_H * 2);

    camera.start();
    std::puts("[diff_detect] running. 'q' quit, 'h' display on/off.\n");

    cv::Mat left, right;
    cv::Mat prev_gray_l, prev_gray_r;
    int64_t ts_ns    = 0;
    int     frame_id = 0;
    bool    show_disp = true;
    double  frame_us  = 0.0;

    while (true) {
        baddy::ScopedTimer frame_timer(&frame_us);

        if (!camera.grab(left, right, ts_ns))
            continue;

        // Convert to grayscale.
        cv::Mat gray_l, gray_r;
        cv::cvtColor(left,  gray_l, cv::COLOR_BGR2GRAY);
        cv::cvtColor(right, gray_r, cv::COLOR_BGR2GRAY);

        // Skip first frame (no previous to diff against).
        if (prev_gray_l.empty()) {
            prev_gray_l = gray_l.clone();
            prev_gray_r = gray_r.clone();
            ++frame_id;
            continue;
        }

        // ── Detect ──
        Detection det_l, det_r;
        cv::Mat diff_l, diff_r, thresh_l, thresh_r;
        double detect_us = 0;
        {
            baddy::ScopedTimer t(&detect_us);
            det_l = detect_diff(prev_gray_l, gray_l, diff_l, thresh_l);
            det_r = detect_diff(prev_gray_r, gray_r, diff_r, thresh_r);
        }

        // Store current as previous for next iteration.
        prev_gray_l = gray_l.clone();
        prev_gray_r = gray_r.clone();

        // ── Build display ──
        double disp_us = 0;
        if (show_disp) {
            baddy::ScopedTimer t(&disp_us);

            const double sx = double(DISP_W) / left.cols;
            const double sy = double(DISP_H) / left.rows;

            cv::Mat disp_l, disp_r, disp_diff_l, disp_diff_r;
            cv::resize(left,  disp_l, {DISP_W, DISP_H});
            cv::resize(right, disp_r, {DISP_W, DISP_H});
            cv::resize(thresh_l, disp_diff_l, {DISP_W, DISP_H});
            cv::resize(thresh_r, disp_diff_r, {DISP_W, DISP_H});

            // Detection circles on camera feeds.
            if (det_l.found) {
                int dx = int(det_l.pixel_x * sx), dy = int(det_l.pixel_y * sy);
                cv::circle(disp_l, {dx, dy}, 8, {0, 255, 0}, 2);
            }
            if (det_r.found) {
                int dx = int(det_r.pixel_x * sx), dy = int(det_r.pixel_y * sy);
                cv::circle(disp_r, {dx, dy}, 8, {0, 255, 0}, 2);
            }

            // FPS readout.
            if (frame_us > 0.0) {
                char fps_text[32];
                std::snprintf(fps_text, sizeof(fps_text), "%.1f FPS", 1e6 / frame_us);
                cv::putText(disp_r, fps_text, {DISP_W - 130, 28},
                            cv::FONT_HERSHEY_SIMPLEX, 0.8, {0, 255, 0}, 2);
            }

            // Assemble 2×2 grid.
            cv::Mat cam_row, diff_row, combined;
            cv::hconcat(disp_l, disp_r, cam_row);

            cv::Mat diff_bgr_l, diff_bgr_r;
            cv::cvtColor(disp_diff_l, diff_bgr_l, cv::COLOR_GRAY2BGR);
            cv::cvtColor(disp_diff_r, diff_bgr_r, cv::COLOR_GRAY2BGR);
            cv::hconcat(diff_bgr_l, diff_bgr_r, diff_row);

            // Threshold label on diff row.
            char thresh_text[64];
            std::snprintf(thresh_text, sizeof(thresh_text),
                          "diff=[%d, %d]  area=[%d, %d]", diff_lo, diff_hi, min_area, max_area);
            cv::putText(diff_row, thresh_text, {10, 30},
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, {0, 255, 0}, 2);

            cv::vconcat(cam_row, diff_row, combined);
            cv::imshow("Diff Detect", combined);
        }

        // Console log every 10 frames.
        if (frame_id % 10 == 0) {
            std::printf("[diff] det=%.0fus disp=%.0fus  total=%.0fus (%.1f FPS)\n",
                        detect_us, disp_us, frame_us, 1e6 / frame_us);
        }

        int key = cv::waitKey(1) & 0xFF;
        if (key == 'q') break;
        if (key == 'h') {
            show_disp = !show_disp;
            std::printf("[diff_detect] display: %s\n", show_disp ? "ON" : "OFF");
        }

        ++frame_id;
    }

    camera.stop();
    cv::destroyAllWindows();
    return 0;
}
