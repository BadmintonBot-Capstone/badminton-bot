// capture_calibration.cpp — Save synchronised stereo image pairs for calibration.
// Reuses StereoCamera so camera config comes from camera.yaml (single source of truth).
// The board comes from config/board.yaml, is detected live in both views, and a pair is only
// saved when enough corners are seen by both cameras.
//
// Usage:
//   ./capture_calibration --config config/ [--output data/calibration_images]
//                         [--exposure us] [--gain db] [--fps n] [--interval s]
//
// Space starts and stops the countdown loop (a shot every --interval seconds, default 3),
// 's' saves a pair right away, 'f' toggles fullscreen, 'q' quits.

#include "core/camera.hpp"
#include "core/config.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect/charuco_detector.hpp>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <map>

namespace {

constexpr int MIN_COMMON = 6;   // same floor as tools/stereo_calibrate.py

cv::aruco::CharucoBoard load_board(const std::string& path)
{
    static const std::map<std::string, cv::aruco::PredefinedDictionaryType> dicts = {
        {"DICT_4X4_50", cv::aruco::DICT_4X4_50},   {"DICT_4X4_100", cv::aruco::DICT_4X4_100},
        {"DICT_5X5_50", cv::aruco::DICT_5X5_50},   {"DICT_5X5_100", cv::aruco::DICT_5X5_100},
        {"DICT_6X6_50", cv::aruco::DICT_6X6_50},   {"DICT_6X6_250", cv::aruco::DICT_6X6_250},
    };
    YAML::Node n = YAML::LoadFile(path);
    auto dict = cv::aruco::getPredefinedDictionary(dicts.at(n["dictionary"].as<std::string>()));
    return cv::aruco::CharucoBoard({n["squares_x"].as<int>(), n["squares_y"].as<int>()},
                                   n["square_mm"].as<float>() / 1000, n["marker_mm"].as<float>() / 1000, dict);
}

// detects on a half size copy for speed, corners come back in full resolution pixels
void detect(const cv::aruco::CharucoDetector& det, const cv::Mat& bgr, std::vector<cv::Point2f>& corners,
            std::vector<int>& ids)
{
    cv::Mat grey, half;
    cv::cvtColor(bgr, grey, cv::COLOR_BGR2GRAY);
    cv::resize(grey, half, {}, 0.5, 0.5, cv::INTER_AREA);
    corners.clear();
    ids.clear();
    det.detectBoard(half, corners, ids);
    for (auto& c : corners) c *= 2.0f;
}

int run(int argc, char** argv);

} // namespace

// an uncaught exception ends a Windows release build silently, so say what went wrong
int main(int argc, char** argv)
{
    try {
        return run(argc, argv);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "[calibration] error: %s\n", e.what());
        return 1;
    }
}

namespace {

int run(int argc, char** argv)
{
    std::string config_dir = "config";
    std::string output_dir = "data/calibration_images";
    // calibration only needs still, clear shots, so these default brighter and slower than camera.yaml
    double exposure = 8333, gain = 12, interval = 3;
    int fps = 20;

    for (int i = 1; i < argc; ++i) {
        auto is = [&](const char* flag) { return !std::strcmp(argv[i], flag) && i + 1 < argc; };
        if      (is("--config"))   config_dir = argv[++i];
        else if (is("--output"))   output_dir = argv[++i];
        else if (is("--exposure")) exposure   = std::atof(argv[++i]);
        else if (is("--gain"))     gain       = std::atof(argv[++i]);
        else if (is("--fps"))      fps        = std::atoi(argv[++i]);
        else if (is("--interval")) interval   = std::atof(argv[++i]);
    }

    std::filesystem::create_directories(output_dir);

    // the folder keeps a copy of the board it was captured with, stereo_calibrate.py reads it from there
    const std::string board_path = config_dir + "/board.yaml";
    std::filesystem::copy_file(board_path, output_dir + "/board.yaml",
                               std::filesystem::copy_options::overwrite_existing);
    auto board = load_board(board_path);
    cv::aruco::CharucoDetector detector(board);

    auto cfg = baddy::load_config(config_dir);
    cfg.camera.exposure_us = exposure;
    cfg.camera.gain_db     = gain;
    cfg.camera.fps         = fps;
    baddy::StereoCamera camera(cfg.camera);
    camera.start();

    std::printf("[calibration] saving to %s/  exposure %.0f us  gain %.1f dB  %d fps\n",
                output_dir.c_str(), exposure, gain, fps);
    std::puts("Space starts/stops the countdown, 's' saves now, 'f' fullscreen, 'q' quits.");

    const char* win = "Calibration Capture";
    cv::namedWindow(win, cv::WINDOW_NORMAL);
    cv::resizeWindow(win, 1920, 600);

    using clock = std::chrono::steady_clock;
    cv::Mat left, right;
    int64_t ts_ns = 0;
    int pair_count = 0, frames = 0;
    std::vector<cv::Point2f> saved_corners;   // every corner saved so far, to show which parts of the image are covered
    auto t0 = clock::now();
    bool looping = false, fullscreen = false;
    auto next_shot = clock::now();
    auto flash_until = clock::now();
    std::string flash_text;
    cv::Scalar flash_col;

    while (true) {
        if (!camera.grab(left, right, ts_ns))
            continue;
        ++frames;

        std::vector<cv::Point2f> cl, cr;
        std::vector<int> il, ir;
        detect(detector, left, cl, il);
        detect(detector, right, cr, ir);
        std::vector<int> sl = il, sr = ir, common;
        std::sort(sl.begin(), sl.end());
        std::sort(sr.begin(), sr.end());
        std::set_intersection(sl.begin(), sl.end(), sr.begin(), sr.end(), std::back_inserter(common));

        auto now = clock::now();
        auto try_save = [&]() {
            if (common.size() < MIN_COMMON) {
                std::printf("  Not saved: only %zu corners seen by both cameras (need %d)\n", common.size(), MIN_COMMON);
                flash_text = "NOT SEEN BY BOTH";
                flash_col  = {0, 0, 255};
            } else {
                char path_l[256], path_r[256];
                std::snprintf(path_l, sizeof(path_l), "%s/left_%02d.png", output_dir.c_str(), pair_count);
                std::snprintf(path_r, sizeof(path_r), "%s/right_%02d.png", output_dir.c_str(), pair_count);
                cv::imwrite(path_l, left);
                cv::imwrite(path_r, right);
                saved_corners.insert(saved_corners.end(), cl.begin(), cl.end());
                std::printf("  Saved pair #%d (%zu corners in both)\n", pair_count, common.size());
                ++pair_count;
                flash_text = "SAVED " + std::to_string(pair_count);
                flash_col  = {0, 255, 0};
            }
            std::fflush(stdout);
            flash_until = now + std::chrono::milliseconds(800);
        };
        if (looping && now >= next_shot) {
            try_save();
            next_shot = now + std::chrono::milliseconds(int(interval * 1000));
        }

        cv::Mat disp_l, disp_r, combined;
        cv::resize(left, disp_l, {960, 600});
        cv::resize(right, disp_r, {960, 600});
        const float s = 960.0f / left.cols;
        for (auto& c : saved_corners) cv::circle(disp_l, c * s, 3, {255, 128, 0}, cv::FILLED);
        for (auto& c : cl) cv::circle(disp_l, c * s, 6, {0, 255, 0}, 2);
        for (auto& c : cr) cv::circle(disp_r, c * s, 6, {0, 255, 0}, 2);
        cv::hconcat(disp_l, disp_r, combined);

        double secs = std::chrono::duration<double>(now - t0).count();
        char label[160];
        std::snprintf(label, sizeof(label), "Pairs saved: %d   corners L %zu  R %zu  both %zu   %.1f fps   %s",
                      pair_count, il.size(), ir.size(), common.size(), frames / std::max(secs, 1e-3),
                      looping ? "space stops" : "space starts the countdown");
        cv::Scalar col = common.size() >= MIN_COMMON ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
        cv::putText(combined, label, {14, 40}, cv::FONT_HERSHEY_SIMPLEX, 1.0, {0, 0, 0}, 6);
        cv::putText(combined, label, {14, 40}, cv::FONT_HERSHEY_SIMPLEX, 1.0, col, 2);

        // big enough to read from where the board is held
        auto big = [&](const std::string& text, cv::Scalar c) {
            int base = 0;
            cv::Size sz = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 9, 24, &base);
            cv::Point org((combined.cols - sz.width) / 2, (combined.rows + sz.height) / 2);
            cv::putText(combined, text, org, cv::FONT_HERSHEY_SIMPLEX, 9, {0, 0, 0}, 40);
            cv::putText(combined, text, org, cv::FONT_HERSHEY_SIMPLEX, 9, c, 24);
        };
        if (now < flash_until) {
            big(flash_text, flash_col);
        } else if (looping) {
            double left_s = std::chrono::duration<double>(next_shot - now).count();
            big(std::to_string(int(std::ceil(std::max(left_s, 0.0)))), col);
        }

        cv::imshow(win, combined);

        int key = cv::waitKey(1) & 0xFF;
        if (key == 'q') break;
        if (key == 's') try_save();
        if (key == ' ') {
            looping = !looping;
            next_shot = now + std::chrono::milliseconds(int(interval * 1000));
        }
        if (key == 'f') {
            fullscreen = !fullscreen;
            cv::setWindowProperty(win, cv::WND_PROP_FULLSCREEN, fullscreen ? cv::WINDOW_FULLSCREEN : cv::WINDOW_NORMAL);
        }
    }

    camera.stop();
    cv::destroyAllWindows();
    std::printf("[calibration] done. %d pairs saved.\n", pair_count);
    return 0;
}

} // namespace
