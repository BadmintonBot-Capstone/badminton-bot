// Real-time stereo pipeline: detect → triangulate → EKF → predict landing.
//
// Usage:
//   ./realtime --config config/ --calibration config/calibration.yaml

#include "core/camera.hpp"
#include "core/config.hpp"
#include "core/stereo.hpp"
#include "core/detector.hpp"
#include "core/tracker.hpp"
#include "core/timing.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <cstdio>
#include <cstring>

int main(int argc, char** argv)
{
    std::string config_dir = "config";
    std::string cal_path   = "config/calibration.yaml";
    bool show_gui = true;

    for (int i = 1; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--config") && i + 1 < argc)
            config_dir = argv[++i];
        else if (!std::strcmp(argv[i], "--calibration") && i + 1 < argc)
            cal_path = argv[++i];
        else if (!std::strcmp(argv[i], "--headless"))
            show_gui = false;
    }

    auto cfg = baddy::load_config(config_dir);
    baddy::load_calibration(cfg, cal_path);

    baddy::StereoCamera camera(cfg.camera);
    baddy::StereoRig    rig(cfg.calibration, cfg.camera.width, cfg.camera.height);
    baddy::Detector     detector(cfg.detection);
    baddy::Tracker      tracker(cfg.ekf);
    baddy::TimingLog    timing;

    camera.start();
    std::puts("[realtime] running.  press 'q' to quit.\n");

    cv::Mat left, right, rect_l, rect_r;
    int64_t ts_ns = 0;
    int frame_id = 0;

    while (true) {
        baddy::FrameTiming ft;

        // ── Acquire ──────────────────────────────────────────────
        {
            baddy::ScopedTimer t(&ft.acquire_us);
            if (!camera.grab(left, right, ts_ns)) continue;
        }

        // ── Rectify ──────────────────────────────────────────────
        {
            baddy::ScopedTimer t(&ft.rectify_us);
            rig.rectify(left, right, rect_l, rect_r);
        }

        // ── Detect ───────────────────────────────────────────────
        baddy::Detection det_l, det_r;
        {
            baddy::ScopedTimer t(&ft.detect_us);
            det_l = detector.detect(rect_l);
            det_r = detector.detect(rect_r);
        }

        // ── Triangulate + Track ──────────────────────────────────
        baddy::TrackerResult result;
        if (det_l.found && det_r.found) {
            baddy::CourtPosition court;
            {
                baddy::ScopedTimer t(&ft.triangulate_us);
                auto cam_xyz = rig.triangulate(det_l.pixel_x, det_l.pixel_y,
                                               det_r.pixel_x, det_r.pixel_y);
                court = rig.to_court(cam_xyz);
            }
            {
                baddy::ScopedTimer t(&ft.ekf_us);
                result = tracker.process_measurement(court, ts_ns);
            }
        } else {
            baddy::ScopedTimer t(&ft.ekf_us);
            result = tracker.process_no_detection(ts_ns);
        }

        ft.total_us = ft.acquire_us + ft.rectify_us + ft.detect_us +
                      ft.triangulate_us + ft.ekf_us;
        timing.record(ft);

        // ── Output ───────────────────────────────────────────────
        if (result.state == baddy::TrackerState::TRACKING && result.prediction.valid) {
            if (frame_id % 10 == 0) {
                std::printf("[rt] landing=[%.2f, %.2f] eta=%.2fs  pipe=%.0fµs\n",
                            result.prediction.position.x,
                            result.prediction.position.z,
                            result.prediction.eta_seconds,
                            ft.total_us);
            }
            // TODO: send result.prediction to robot MCU via wireless link.
        }

        // ── Display (optional) ───────────────────────────────────
        if (show_gui) {
            cv::Mat disp_l, disp_r;
            cv::resize(rect_l, disp_l, {640, 400});
            cv::resize(rect_r, disp_r, {640, 400});
            cv::Mat combined;
            cv::hconcat(disp_l, disp_r, combined);

            const char* state_str = "???";
            switch (result.state) {
                case baddy::TrackerState::SEARCHING: state_str = "SEARCHING"; break;
                case baddy::TrackerState::TRACKING:  state_str = "TRACKING";  break;
                case baddy::TrackerState::LOST:      state_str = "LOST";      break;
            }
            cv::putText(combined, state_str, {10, 30},
                        cv::FONT_HERSHEY_SIMPLEX, 0.8, {0, 255, 0}, 2);

            if (result.prediction.valid) {
                char buf[128];
                std::snprintf(buf, sizeof(buf), "Landing: [%.1f, %.1f] ETA=%.2fs",
                              result.prediction.position.x,
                              result.prediction.position.z,
                              result.prediction.eta_seconds);
                cv::putText(combined, buf, {10, 60},
                            cv::FONT_HERSHEY_SIMPLEX, 0.6, {0, 255, 255}, 2);
            }

            cv::imshow("Realtime", combined);
            if ((cv::waitKey(1) & 0xFF) == 'q') break;
        }

        // Periodic timing summary.
        if (frame_id > 0 && frame_id % 500 == 0)
            std::printf("[timing] %s\n", timing.summary().c_str());

        ++frame_id;
    }

    camera.stop();
    cv::destroyAllWindows();
    std::printf("\n[timing] final: %s\n", timing.summary().c_str());
    return 0;
}
