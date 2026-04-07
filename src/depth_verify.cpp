// depth_verify.cpp — Live HSV tuning + stereo depth verification.
// Shows detection contours, HSV masks, epipolar lines, triangulated 3D position.
// C++ port of old/depth_verify.py, using StereoCamera for config from camera.yaml.
//
// Usage:
//   ./depth_verify [--config config/] [--calibration config/calibration.yaml]
//
// Keys: 'q' quit, 'p' print current HSV values, 'r' toggle raw/rectified,
//        'h' toggle display on/off (to measure pipeline-only latency)

#include "core/camera.hpp"
#include "core/config.hpp"
#include "core/stereo.hpp"
#include "core/detector.hpp"
#include "core/timing.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <deque>

static int h_lo, s_lo, v_lo;
static int h_hi, s_hi, v_hi;

static constexpr int    DISP_W    = 640;
static constexpr int    DISP_H    = 400;
static constexpr int    TRAIL_LEN = 30;
static constexpr double DEPTH_MIN = 1.2;
static constexpr double DEPTH_MAX = 13.0;

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
    std::puts("[depth_verify] running. 'q' quit, 'p' print HSV, 'r' raw/rect, 'h' display on/off.\n");

    cv::Mat left, right, rect_l, rect_r;
    int64_t ts_ns  = 0;
    int     frame_id  = 0;
    bool    show_raw  = false;
    bool    show_disp = true;
    double  last_x = 0.0, last_y = 0.0, last_z = 0.0;
    double  frame_us  = 0.0;
    std::deque<cv::Point> trail_l, trail_r;

    while (true) {
        baddy::ScopedTimer frame_timer(&frame_us);

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

        // ── Triangulate ──
        double tri_us    = 0;
        double disparity = 0.0;
        double epi_err   = -1.0;
        bool   both_det  = false;
        double court_x = 0, court_y = 0, court_z = 0;
        char   depth_text[256] = "Depth: ---";

        if (det_l.found && det_r.found) {
            baddy::ScopedTimer t(&tri_us);
            auto cam_xyz = rig.triangulate(det_l.pixel_x, det_l.pixel_y,
                                           det_r.pixel_x, det_r.pixel_y);
            auto court = rig.to_court(cam_xyz);
            disparity = det_l.pixel_x - det_r.pixel_x;
            epi_err   = std::abs(det_l.pixel_y - det_r.pixel_y);
            last_x = cam_xyz[0]; last_y = cam_xyz[1]; last_z = cam_xyz[2];
            court_x = court.x; court_y = court.y; court_z = court.z;
            both_det = true;

            std::snprintf(depth_text, sizeof(depth_text),
                          "[%.3f, %.3f, %.3f] m  disp=%.1f px",
                          court.x, court.y, court.z, disparity);
        }

        // ── Build display ──
        double disp_us = 0;
        if (show_disp) {
            baddy::ScopedTimer t(&disp_us);

            // ── HSV masks (both cameras) ──
            cv::Mat hsv_l, hsv_r, mask_l, mask_r;
            cv::cvtColor(rect_l, hsv_l, cv::COLOR_BGR2HSV);
            cv::cvtColor(rect_r, hsv_r, cv::COLOR_BGR2HSV);
            cv::Scalar lo(h_lo, s_lo, v_lo), hi(h_hi, s_hi, v_hi);
            cv::inRange(hsv_l, lo, hi, mask_l);
            cv::inRange(hsv_r, lo, hi, mask_r);

            const double sx = double(DISP_W) / rect_l.cols;
            const double sy = double(DISP_H) / rect_l.rows;

            // Update motion trails in display-space coordinates.
            if (det_l.found) {
                trail_l.push_back({int(det_l.pixel_x * sx), int(det_l.pixel_y * sy)});
                if (static_cast<int>(trail_l.size()) > TRAIL_LEN) trail_l.pop_front();
            }
            if (det_r.found) {
                trail_r.push_back({int(det_r.pixel_x * sx), int(det_r.pixel_y * sy)});
                if (static_cast<int>(trail_r.size()) > TRAIL_LEN) trail_r.pop_front();
            }

            // Top row: raw or rectified depending on toggle.
            cv::Mat disp_l, disp_r, disp_mask_l, disp_mask_r;
            if (show_raw) {
                cv::resize(left,  disp_l, {DISP_W, DISP_H});
                cv::resize(right, disp_r, {DISP_W, DISP_H});
            } else {
                cv::resize(rect_l, disp_l, {DISP_W, DISP_H});
                cv::resize(rect_r, disp_r, {DISP_W, DISP_H});
            }
            cv::resize(mask_l, disp_mask_l, {DISP_W, DISP_H});
            cv::resize(mask_r, disp_mask_r, {DISP_W, DISP_H});

            // Overlays only in rectified mode (detections run on rectified frames).
            if (!show_raw) {
                // Motion trails: older segments are dim orange, newer are bright yellow.
                auto draw_trail = [](cv::Mat& img, const std::deque<cv::Point>& trail) {
                    for (int i = 1; i < static_cast<int>(trail.size()); ++i) {
                        double alpha = double(i) / TRAIL_LEN;
                        cv::Scalar col(0, int(200 * alpha), int(255 * alpha));
                        cv::line(img, trail[i-1], trail[i], col, 2);
                    }
                };
                draw_trail(disp_l, trail_l);
                draw_trail(disp_r, trail_r);

                // Detection circles + epipolar lines.
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
            }

            // RAW / RECT label (top-right of left panel).
            cv::putText(disp_l, show_raw ? "RAW" : "RECT", {DISP_W - 75, 28},
                        cv::FONT_HERSHEY_SIMPLEX, 0.8, {0, 200, 255}, 2);

            // FPS readout (top-right of right panel).
            if (frame_us > 0.0) {
                char fps_text[32];
                std::snprintf(fps_text, sizeof(fps_text), "%.1f FPS", 1e6 / frame_us);
                cv::putText(disp_r, fps_text, {DISP_W - 130, 28},
                            cv::FONT_HERSHEY_SIMPLEX, 0.8, {0, 255, 0}, 2);
            }

            // ── Assemble cam_row ──
            cv::Mat cam_row, mask_row_l, mask_row_r, mask_row, combined;
            cv::hconcat(disp_l, disp_r, cam_row);
            cv::cvtColor(disp_mask_l, mask_row_l, cv::COLOR_GRAY2BGR);
            cv::cvtColor(disp_mask_r, mask_row_r, cv::COLOR_GRAY2BGR);
            cv::hconcat(mask_row_l, mask_row_r, mask_row);

            // Small depth text (top-left).
            cv::Scalar text_color = both_det ? cv::Scalar(0, 0, 0) : cv::Scalar(0, 0, 255);
            cv::putText(cam_row, depth_text, {10, 30},
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, text_color, 2);

            // Epipolar error (below depth text).
            if (epi_err >= 0.0) {
                char epi_text[64];
                std::snprintf(epi_text, sizeof(epi_text), "Epi err: %.1f px", epi_err);
                cv::putText(cam_row, epi_text, {10, 60},
                            cv::FONT_HERSHEY_SIMPLEX, 0.7, {0, 255, 255}, 2);
            }

            // Big XYZ readout (centered near bottom of cam_row).
            {
                char xyz_text[64];
                std::snprintf(xyz_text, sizeof(xyz_text), "X:%.2f  Y:%.2f  Z:%.2f m",
                              last_x, last_y, last_z);
                int baseline = 0;
                auto sz = cv::getTextSize(xyz_text, cv::FONT_HERSHEY_DUPLEX, 1.5, 3, &baseline);
                int tx = (cam_row.cols - sz.width) / 2;
                int ty = cam_row.rows - 15;
                cv::putText(cam_row, xyz_text, {tx, ty},
                            cv::FONT_HERSHEY_DUPLEX, 1.5, {0, 255, 255}, 3);
            }

            // Depth bar (right edge of cam_row, 1.2–13 m range).
            {
                constexpr int BAR_W   = 20;
                constexpr int BAR_X   = DISP_W * 2 - BAR_W - 5;
                constexpr int BAR_TOP = 10;
                constexpr int BAR_BOT = DISP_H - 10;
                constexpr int BAR_H   = BAR_BOT - BAR_TOP;

                cv::rectangle(cam_row, {BAR_X, BAR_TOP}, {BAR_X + BAR_W, BAR_BOT},
                              {180, 180, 180}, 1);

                if (last_z > 0.0) {
                    double norm     = std::clamp((last_z - DEPTH_MIN) / (DEPTH_MAX - DEPTH_MIN), 0.0, 1.0);
                    int    fill_h   = int(norm * BAR_H);
                    int    fill_top = BAR_BOT - fill_h;
                    double r = 255.0 * norm;
                    double g = 255.0 * (1.0 - norm);
                    cv::rectangle(cam_row, {BAR_X, fill_top}, {BAR_X + BAR_W, BAR_BOT},
                                  {0.0, g, r}, cv::FILLED);
                }
            }

            // HSV bounds on mask row.
            char hsv_text[128];
            std::snprintf(hsv_text, sizeof(hsv_text),
                          "HSV: [%d,%d,%d] - [%d,%d,%d]",
                          h_lo, s_lo, v_lo, h_hi, s_hi, v_hi);
            cv::putText(mask_row, hsv_text, {10, 30},
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, {0, 255, 0}, 2);

            cv::vconcat(cam_row, mask_row, combined);
            cv::imshow("Depth Verify", combined);
        } // end display timer

        // Console log every 10 frames.
        if (frame_id % 10 == 0) {
            if (both_det) {
                std::printf("[depth] xyz=[%.3f, %.3f, %.3f] m  "
                            "rect=%.0fus det=%.0fus tri=%.0fus disp=%.0fus  total=%.0fus (%.1f FPS)\n",
                            court_x, court_y, court_z,
                            rectify_us, detect_us, tri_us, disp_us,
                            frame_us, 1e6 / frame_us);
            } else {
                std::printf("[depth] no detection  "
                            "rect=%.0fus det=%.0fus disp=%.0fus  total=%.0fus (%.1f FPS)\n",
                            rectify_us, detect_us, disp_us,
                            frame_us, 1e6 / frame_us);
            }
        }

        int key = cv::waitKey(1) & 0xFF;
        if (key == 'q') break;
        if (key == 'p') {
            std::printf("\nHSV low:  [%d, %d, %d]\nHSV high: [%d, %d, %d]\n\n",
                        h_lo, s_lo, v_lo, h_hi, s_hi, v_hi);
        }
        if (key == 'r') {
            show_raw = !show_raw;
            std::printf("[depth_verify] display: %s\n", show_raw ? "RAW" : "RECTIFIED");
        }
        if (key == 'h') {
            show_disp = !show_disp;
            std::printf("[depth_verify] display: %s\n", show_disp ? "ON" : "OFF");
        }

        ++frame_id;
    }

    camera.stop();
    cv::destroyAllWindows();
    return 0;
}
