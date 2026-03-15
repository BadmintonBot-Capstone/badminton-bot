// Reads a trajectory CSV (paper dataset or own captures), feeds each frame's
// XYZ through the Tracker, and reports EKF accuracy + landing prediction error.
//
// Usage:
//   ./replay_trajectory --config config/ --trajectory path/to/Model3D.csv

#include "core/config.hpp"
#include "core/tracker.hpp"
#include "core/timing.hpp"
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cstdio>
#include <cstring>
#include <map>

struct CsvRow {
    int    frame      = 0;
    int    visibility = 0;
    double timestamp  = 0.0;
    std::map<std::string, double> columns;
};

static std::vector<CsvRow> load_csv(const std::string& path)
{
    std::ifstream file(path);
    if (!file.is_open())
        throw std::runtime_error("Cannot open CSV: " + path);

    std::string header_line;
    std::getline(file, header_line);

    // Parse header names.
    std::vector<std::string> headers;
    {
        std::stringstream ss(header_line);
        std::string token;
        while (std::getline(ss, token, ','))
            headers.push_back(token);
    }

    std::vector<CsvRow> rows;
    std::string line;
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        std::string token;
        CsvRow row;
        for (std::size_t i = 0; i < headers.size() && std::getline(ss, token, ','); ++i) {
            if (headers[i] == "Frame")      row.frame = std::stoi(token);
            else if (headers[i] == "Visibility") row.visibility = std::stoi(token);
            else if (headers[i] == "Timestamp")  row.timestamp = std::stod(token);
            else row.columns[headers[i]] = std::stod(token);
        }
        rows.push_back(row);
    }
    return rows;
}

static void print_usage()
{
    std::puts("Usage: replay_trajectory --config <dir> --trajectory <csv> [--vicon <csv>]");
}

int main(int argc, char** argv)
{
    std::string config_dir = "config";
    std::string trajectory_path;
    std::string vicon_path;

    for (int i = 1; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--config") && i + 1 < argc)
            config_dir = argv[++i];
        else if (!std::strcmp(argv[i], "--trajectory") && i + 1 < argc)
            trajectory_path = argv[++i];
        else if (!std::strcmp(argv[i], "--vicon") && i + 1 < argc)
            vicon_path = argv[++i];
        else { print_usage(); return 1; }
    }

    if (trajectory_path.empty()) { print_usage(); return 1; }

    // ── Load config ──────────────────────────────────────────────
    auto cfg = baddy::load_config(config_dir);
    const auto& ekf_cfg = cfg.ekf;

    std::printf("alpha=%.3f  sigma_p=%.3f  sigma_v=%.1f  sigma_cam=[%.3f, %.3f, %.3f]\n",
                ekf_cfg.alpha, ekf_cfg.sigma_p, ekf_cfg.sigma_v,
                ekf_cfg.sigma_cam_x, ekf_cfg.sigma_cam_y, ekf_cfg.sigma_cam_z);
    std::printf("target_y=%.2f  max_coast=%d\n\n",
                ekf_cfg.target_intercept_y, ekf_cfg.max_coast_frames);

    // ── Load trajectory ──────────────────────────────────────────
    auto rows = load_csv(trajectory_path);
    std::printf("Loaded %zu rows from %s\n", rows.size(), trajectory_path.c_str());

    // ── Coordinate remap ─────────────────────────────────────────
    auto remap = [&](const CsvRow& r) -> baddy::CourtPosition {
        double x = r.columns.at(ekf_cfg.x_source) + ekf_cfg.x_offset;
        double y = r.columns.at(ekf_cfg.y_source) + ekf_cfg.y_offset;
        double z = r.columns.at(ekf_cfg.z_source) + ekf_cfg.z_offset;
        return {x, y, z};
    };

    // ── Run tracker ──────────────────────────────────────────────
    baddy::Tracker tracker(ekf_cfg);
    baddy::TimingLog timing;

    baddy::CourtPosition last_pos{};
    baddy::LandingPrediction last_prediction{};

    for (const auto& row : rows) {
        if (!row.visibility) {
            auto ts_ns = static_cast<int64_t>(row.timestamp * 1e9);
            tracker.process_no_detection(ts_ns);
            continue;
        }

        baddy::CourtPosition pos = remap(row);
        auto ts_ns = static_cast<int64_t>(row.timestamp * 1e9);

        baddy::FrameTiming ft;
        {
            baddy::ScopedTimer t(&ft.ekf_us);
            auto result = tracker.process_measurement(pos, ts_ns);
            last_prediction = result.prediction;
        }
        ft.total_us = ft.ekf_us;
        timing.record(ft);

        last_pos = pos;

        // Print periodic status.
        if (row.frame % 20 == 0 && tracker.state() == baddy::TrackerState::TRACKING) {
            const auto& s = tracker.state() == baddy::TrackerState::TRACKING
                          ? last_prediction : baddy::LandingPrediction{};
            std::printf("frame %3d  pos=[%6.2f %6.2f %6.2f]  "
                        "ekf_vel=[%6.1f %6.1f %6.1f]",
                        row.frame, pos.x, pos.y, pos.z,
                        last_prediction.valid ? 0.0 : 0.0, // placeholder
                        0.0, 0.0);
            if (last_prediction.valid) {
                std::printf("  landing=[%5.2f %5.2f] eta=%.2fs",
                            last_prediction.position.x,
                            last_prediction.position.z,
                            last_prediction.eta_seconds);
            }
            std::puts("");
        }
    }

    // ── Summary ──────────────────────────────────────────────────
    std::puts("\n=== Timing ===");
    std::printf("%s\n", timing.summary().c_str());

    // Report where the birdie actually ended vs where the predictor said.
    std::puts("\n=== Landing Prediction Accuracy ===");
    std::printf("Final observed position:  [%.2f, %.2f, %.2f]\n",
                last_pos.x, last_pos.y, last_pos.z);
    if (last_prediction.valid) {
        const double err_x = last_prediction.position.x - last_pos.x;
        const double err_z = last_prediction.position.z - last_pos.z;
        const double err = std::sqrt(err_x * err_x + err_z * err_z);
        std::printf("Predicted landing (XZ):  [%.2f, %.2f]  eta=%.2fs\n",
                    last_prediction.position.x, last_prediction.position.z,
                    last_prediction.eta_seconds);
        std::printf("Horizontal error:        %.3f m\n", err);
    } else {
        std::puts("No valid landing prediction at end of trajectory.");
    }

    return 0;
}
