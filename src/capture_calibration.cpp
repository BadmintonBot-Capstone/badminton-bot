// capture_calibration.cpp — Save synchronised stereo image pairs for calibration.
// Reuses StereoCamera so camera config comes from camera.yaml (single source of truth).
//
// Usage:
//   ./capture_calibration --config config/ [--output data/calibration_images]
//
// Press 's' to save a pair, 'q' to quit.

#include "core/camera.hpp"
#include "core/config.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <cstdio>
#include <cstring>
#include <filesystem>

int main(int argc, char** argv)
{
    std::string config_dir = "config";
    std::string output_dir = "data/calibration_images";

    for (int i = 1; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--config") && i + 1 < argc)
            config_dir = argv[++i];
        else if (!std::strcmp(argv[i], "--output") && i + 1 < argc)
            output_dir = argv[++i];
    }

    std::filesystem::create_directories(output_dir);

    auto cfg = baddy::load_config(config_dir);
    baddy::StereoCamera camera(cfg.camera);
    camera.start();

    std::printf("[calibration] saving to %s/\n", output_dir.c_str());
    std::puts("Press 's' to save a pair, 'q' to quit.");

    cv::Mat left, right;
    int64_t ts_ns = 0;
    int pair_count = 0;

    while (true) {
        if (!camera.grab(left, right, ts_ns))
            continue;

        cv::Mat disp_l, disp_r, combined;
        cv::resize(left, disp_l, {640, 400});
        cv::resize(right, disp_r, {640, 400});
        cv::hconcat(disp_l, disp_r, combined);

        char label[64];
        std::snprintf(label, sizeof(label), "Pairs saved: %d", pair_count);
        cv::putText(combined, label, {10, 30},
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, {0, 255, 0}, 2);

        cv::imshow("Calibration Capture", combined);

        int key = cv::waitKey(1) & 0xFF;
        if (key == 'q') break;
        if (key == 's') {
            char path_l[256], path_r[256];
            std::snprintf(path_l, sizeof(path_l), "%s/left_%02d.png", output_dir.c_str(), pair_count);
            std::snprintf(path_r, sizeof(path_r), "%s/right_%02d.png", output_dir.c_str(), pair_count);
            cv::imwrite(path_l, left);
            cv::imwrite(path_r, right);
            std::printf("  Saved pair #%d\n", pair_count);
            ++pair_count;
        }
    }

    camera.stop();
    cv::destroyAllWindows();
    std::printf("[calibration] done. %d pairs saved.\n", pair_count);
    return 0;
}
