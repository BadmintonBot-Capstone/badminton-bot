// Live stereo capture: detect the birdie, triangulate, and save trajectory CSVs.
//
// Usage:
//   ./capture_trajectories --config config/ --calibration config/calibration.yaml --output data/

#include "core/camera.hpp"
#include "core/config.hpp"
#include "core/stereo.hpp"
#include "core/detector.hpp"
#include "core/timing.hpp"
#include "core/types.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <fstream>
#include <cstdio>
#include <cstring>
#include <chrono>
#include <filesystem>

struct TrajectoryRow {
    int     frame;
    double  timestamp;
    bool    detected;
    double  x, y, z;
    double  disp;
    double  pl_x, pl_y, pr_x, pr_y;
};

static void save_csv(const std::string& path, const std::vector<TrajectoryRow>& rows)
{
    std::ofstream f(path);
    f << "Frame,Visibility,X,Y,Z,Timestamp,Disparity,PL_X,PL_Y,PR_X,PR_Y\n";
    for (const auto& r : rows) {
        char buf[256];
        std::snprintf(buf, sizeof(buf),
                      "%d,%d,%.6f,%.6f,%.6f,%.9f,%.2f,%.1f,%.1f,%.1f,%.1f\n",
                      r.frame, r.detected ? 1 : 0,
                      r.x, r.y, r.z, r.timestamp, r.disp,
                      r.pl_x, r.pl_y, r.pr_x, r.pr_y);
        f << buf;
    }
    std::printf("[capture] saved %zu rows to %s\n", rows.size(), path.c_str());
}

int main(int argc, char** argv)
{
    std::string config_dir = "config";
    std::string cal_path   = "config/calibration.yaml";
    std::string output_dir = "data";

    for (int i = 1; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--config") && i + 1 < argc)
            config_dir = argv[++i];
        else if (!std::strcmp(argv[i], "--calibration") && i + 1 < argc)
            cal_path = argv[++i];
        else if (!std::strcmp(argv[i], "--output") && i + 1 < argc)
            output_dir = argv[++i];
    }

    auto cfg = baddy::load_config(config_dir);
    baddy::load_calibration(cfg, cal_path);

    baddy::StereoCamera camera(cfg.camera);
    baddy::StereoRig    rig(cfg.calibration, cfg.camera.width, cfg.camera.height);
    baddy::Detector     detector(cfg.detection);

    std::filesystem::create_directories(output_dir);

    camera.start();
    std::puts("[capture] press 'q' to quit.  Trajectories auto-save on track loss.\n");

    cv::Mat left, right, rect_l, rect_r;
    int64_t ts_ns = 0;
    int frame_id = 0;
    int file_count = 0;
    bool recording = false;
    std::vector<TrajectoryRow> buffer;
    int miss_count = 0;
    constexpr int kMissThreshold = 30;

    while (true) {
        if (!camera.grab(left, right, ts_ns)) continue;

        rig.rectify(left, right, rect_l, rect_r);

        auto det_l = detector.detect(rect_l);
        auto det_r = detector.detect(rect_r);

        TrajectoryRow row{};
        row.frame = frame_id;
        row.timestamp = static_cast<double>(ts_ns) * 1e-9;

        if (det_l.found && det_r.found) {
            auto cam_xyz = rig.triangulate(det_l.pixel_x, det_l.pixel_y,
                                           det_r.pixel_x, det_r.pixel_y);
            auto court = rig.to_court(cam_xyz);

            row.detected = true;
            row.x = court.x; row.y = court.y; row.z = court.z;
            row.disp = det_l.pixel_x - det_r.pixel_x;
            row.pl_x = det_l.pixel_x; row.pl_y = det_l.pixel_y;
            row.pr_x = det_r.pixel_x; row.pr_y = det_r.pixel_y;

            if (!recording) {
                recording = true;
                buffer.clear();
                std::puts("[capture] recording started");
            }
            miss_count = 0;
        } else {
            if (recording) {
                ++miss_count;
                if (miss_count >= kMissThreshold) {
                    auto name = output_dir + "/trajectory_" +
                                std::to_string(file_count++) + ".csv";
                    save_csv(name, buffer);
                    recording = false;
                    miss_count = 0;
                }
            }
        }

        if (recording) buffer.push_back(row);

        // Display.
        cv::Mat disp_l, disp_r;
        cv::resize(rect_l, disp_l, {640, 400});
        cv::resize(rect_r, disp_r, {640, 400});
        cv::Mat combined;
        cv::hconcat(disp_l, disp_r, combined);

        const char* status = recording ? "REC" : "IDLE";
        cv::putText(combined, status, {10, 30},
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, {0, 0, 255}, 2);

        cv::imshow("Capture", combined);
        if ((cv::waitKey(1) & 0xFF) == 'q') break;

        ++frame_id;
    }

    // Save any remaining buffer.
    if (recording && !buffer.empty()) {
        auto name = output_dir + "/trajectory_" + std::to_string(file_count++) + ".csv";
        save_csv(name, buffer);
    }

    camera.stop();
    cv::destroyAllWindows();
    return 0;
}
