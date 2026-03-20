// Reads a trajectory CSV (paper dataset or own captures), feeds each frame's
// XYZ through the Tracker, and reports EKF tracking accuracy and landing
// prediction convergence over time.
//
// Usage:
//   ./replay_trajectory --config <dir> --trajectory <csv>

#include "core/config.hpp"
#include "core/tracker.hpp"
#include "core/timing.hpp"
#include "core/types.hpp"
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <map>
#include <numeric>
#include <algorithm>

// ── CSV loading ───────────────────────────────────────────────────────────────

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
            if (headers[i] == "Frame")           row.frame = std::stoi(token);
            else if (headers[i] == "Visibility") row.visibility = std::stoi(token);
            else if (headers[i] == "Timestamp")  row.timestamp = std::stod(token);
            else row.columns[headers[i]] = std::stod(token);
        }
        rows.push_back(row);
    }
    return rows;
}

// ── Per-frame record ──────────────────────────────────────────────────────────

struct FrameRecord {
    int abs_frame     = 0;   // CSV frame index
    int tracking_frame = 0;  // frames since EKF initialised (0-based)

    // Ground truth (Vicon)
    double gt_x = 0, gt_y = 0, gt_z = 0;

    // EKF estimate
    double ekf_x = 0, ekf_y = 0, ekf_z = 0;
    double ekf_vx = 0, ekf_vy = 0, ekf_vz = 0;

    double pos_error_m = 0;   // |ekf_pos - gt_pos|

    double timestamp_s = 0;   // CSV timestamp of this frame

    // Landing prediction at this frame
    bool   pred_valid  = false;
    double pred_x      = 0, pred_z = 0;
    double pred_eta_s  = 0;
    double pred_error_m = 0;  // horizontal distance from prediction to gt_landing
    double true_eta_s  = 0;   // actual time remaining (gt_landing_ts - this_ts)
    double eta_error_s = 0;   // pred_eta - true_eta
};

// ── Helpers ───────────────────────────────────────────────────────────────────

static void print_usage()
{
    std::puts("Usage: replay_trajectory --config <dir> --trajectory <csv>");
}

static double rmse(const std::vector<double>& v)
{
    double sq = 0;
    for (double x : v) sq += x * x;
    return std::sqrt(sq / static_cast<double>(v.size()));
}

// ── Main ──────────────────────────────────────────────────────────────────────

int main(int argc, char** argv)
{
    std::string config_dir = "config";
    std::string trajectory_path;

    for (int i = 1; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--config") && i + 1 < argc)
            config_dir = argv[++i];
        else if (!std::strcmp(argv[i], "--trajectory") && i + 1 < argc)
            trajectory_path = argv[++i];
        else { print_usage(); return 1; }
    }

    if (trajectory_path.empty()) { print_usage(); return 1; }

    // ── Load config ───────────────────────────────────────────────────────────
    auto cfg = baddy::load_config(config_dir);
    const auto& ekf_cfg = cfg.ekf;

    std::puts("=== EKF Config ===");
    std::printf("  alpha=%.3f      (drag coeff; wrong value → systematic landing error)\n",
                ekf_cfg.alpha);
    std::printf("  sigma_p=%.3f m  (position process noise; higher → faster but noisier convergence)\n",
                ekf_cfg.sigma_p);
    std::printf("  sigma_v=%.1f m/s (velocity process noise; higher → adapts to unmodeled forces)\n",
                ekf_cfg.sigma_v);
    std::printf("  sigma_cam=[%.3f, %.3f, %.3f] m  (meas noise x/y/z; lower → trust camera more)\n",
                ekf_cfg.sigma_cam_x, ekf_cfg.sigma_cam_y, ekf_cfg.sigma_cam_z);
    std::printf("  initial_P_vel=%.0f  (high → velocity converges fast from first 2 detections)\n",
                ekf_cfg.initial_p_velocity);
    std::printf("  target_y=%.2f  max_coast=%d\n\n",
                ekf_cfg.target_intercept_y, ekf_cfg.max_coast_frames);

    // ── Load trajectory ───────────────────────────────────────────────────────
    auto rows = load_csv(trajectory_path);
    std::printf("Loaded %zu rows from %s\n\n", rows.size(), trajectory_path.c_str());

    // ── Coordinate remap ──────────────────────────────────────────────────────
    auto remap = [&](const CsvRow& r) -> baddy::CourtPosition {
        double x = r.columns.at(ekf_cfg.x_source) + ekf_cfg.x_offset;
        double y = r.columns.at(ekf_cfg.y_source) + ekf_cfg.y_offset;
        double z = r.columns.at(ekf_cfg.z_source) + ekf_cfg.z_offset;
        return {x, y, z};
    };

    // ── Find ground-truth landing proxy ───────────────────────────────────────
    // The last visible Vicon position serves as the reference for prediction
    // error. Note: this is a proxy — the dataset may not extend all the way
    // to y=0.
    baddy::CourtPosition gt_landing{};
    double gt_landing_ts = 0;
    for (auto it = rows.rbegin(); it != rows.rend(); ++it) {
        if (it->visibility) {
            gt_landing = remap(*it);
            gt_landing_ts = it->timestamp;
            break;
        }
    }
    std::printf("GT landing proxy (last visible pos): [%.2f, %.2f, %.2f] at t=%.4fs\n\n",
                gt_landing.x, gt_landing.y, gt_landing.z, gt_landing_ts);

    // ── Run tracker ───────────────────────────────────────────────────────────
    baddy::Tracker tracker(ekf_cfg);
    baddy::TimingLog timing;

    std::vector<FrameRecord> records;
    int tracking_frame = 0;

    for (const auto& row : rows) {
        auto ts_ns = static_cast<int64_t>(row.timestamp * 1e9);

        if (!row.visibility) {
            tracker.process_no_detection(ts_ns);
            continue;
        }

        baddy::CourtPosition pos = remap(row);
        baddy::TrackerResult result;

        {
            baddy::FrameTiming ft;
            baddy::ScopedTimer t(&ft.ekf_us);
            result = tracker.process_measurement(pos, ts_ns);
            ft.total_us = ft.ekf_us;
            timing.record(ft);
        }

        if (result.state != baddy::TrackerState::TRACKING)
            continue;

        FrameRecord rec;
        rec.abs_frame      = row.frame;
        rec.tracking_frame = tracking_frame++;

        rec.gt_x = pos.x;  rec.gt_y = pos.y;  rec.gt_z = pos.z;

        rec.ekf_x  = result.ekf_state[baddy::PX];
        rec.ekf_y  = result.ekf_state[baddy::PY];
        rec.ekf_z  = result.ekf_state[baddy::PZ];
        rec.ekf_vx = result.ekf_state[baddy::VX];
        rec.ekf_vy = result.ekf_state[baddy::VY];
        rec.ekf_vz = result.ekf_state[baddy::VZ];

        rec.pos_error_m = std::sqrt(
            std::pow(rec.ekf_x - rec.gt_x, 2) +
            std::pow(rec.ekf_y - rec.gt_y, 2) +
            std::pow(rec.ekf_z - rec.gt_z, 2));

        rec.timestamp_s = row.timestamp;
        rec.true_eta_s  = gt_landing_ts - row.timestamp;

        if (result.prediction.valid) {
            rec.pred_valid = true;
            rec.pred_x     = result.prediction.position.x;
            rec.pred_z     = result.prediction.position.z;
            rec.pred_eta_s = result.prediction.eta_seconds;
            rec.pred_error_m = std::sqrt(
                std::pow(rec.pred_x - gt_landing.x, 2) +
                std::pow(rec.pred_z - gt_landing.z, 2));
            rec.eta_error_s = rec.pred_eta_s - rec.true_eta_s;
        }

        records.push_back(rec);
    }

    if (records.empty()) {
        std::puts("No TRACKING frames recorded. Check trajectory file and config.");
        return 1;
    }

    // ── Section A: EKF tracking quality ──────────────────────────────────────
    std::puts("=== EKF Position Error vs Ground Truth ===");
    std::printf("%-6s %-7s  %-22s  %-22s  %s\n",
                "Frame", "Track#", "GT (x, y, z)", "EKF (x, y, z)", "Err(m)");

    int step = (records.size() > 50) ? 5 : 1;
    for (const auto& r : records) {
        if (r.tracking_frame % step != 0) continue;
        std::printf("%-6d %-7d  (%6.2f,%6.2f,%6.2f)   (%6.2f,%6.2f,%6.2f)   %.3f\n",
                    r.abs_frame, r.tracking_frame,
                    r.gt_x, r.gt_y, r.gt_z,
                    r.ekf_x, r.ekf_y, r.ekf_z,
                    r.pos_error_m);
    }

    std::vector<double> pos_errors;
    for (const auto& r : records) pos_errors.push_back(r.pos_error_m);
    double mean_err = std::accumulate(pos_errors.begin(), pos_errors.end(), 0.0) / pos_errors.size();
    double max_err  = *std::max_element(pos_errors.begin(), pos_errors.end());
    std::printf("Summary: mean=%.3fm  max=%.3fm  RMSE=%.3fm\n",
                mean_err, max_err, rmse(pos_errors));

    // ── Section B: Prediction convergence ────────────────────────────────────
    std::puts("\n=== Landing Prediction Convergence ===");
    std::printf("%-7s  %-8s  %-8s  %-8s  %-14s  %s\n",
                "Track#", "ETA(s)", "TrueETA", "ETAerr", "Pred (x, z)", "Err vs GT-landing(m)");

    for (const auto& r : records) {
        if (r.tracking_frame % 5 != 0) continue;
        if (!r.pred_valid) {
            std::printf("%-7d  %-8s  %-8.2f  %-8s  %-14s  %s\n",
                        r.tracking_frame, "--", r.true_eta_s, "--", "--", "--");
            continue;
        }
        std::printf("%-7d  %-8.2f  %-8.2f  %+7.3f   (%6.2f,%6.2f)  %.3f\n",
                    r.tracking_frame, r.pred_eta_s, r.true_eta_s, r.eta_error_s,
                    r.pred_x, r.pred_z, r.pred_error_m);
    }

    // ── Section C: Summary ────────────────────────────────────────────────────
    std::puts("\n=== Summary ===");
    std::printf("Tracking frames:        %zu\n", records.size());
    std::printf("EKF position RMSE:      %.3f m\n", rmse(pos_errors));
    std::printf("EKF position max error: %.3f m\n", max_err);

    std::puts("\nLanding prediction:");
    // Find first and last valid predictions
    const FrameRecord* first_pred = nullptr;
    const FrameRecord* final_pred = nullptr;
    for (const auto& r : records) {
        if (r.pred_valid) {
            if (!first_pred) first_pred = &r;
            final_pred = &r;
        }
    }

    if (first_pred && final_pred) {
        std::printf("  First prediction error: %.3f m  (tracking frame %d)\n",
                    first_pred->pred_error_m, first_pred->tracking_frame);
        std::printf("  Final prediction error: %.3f m  (tracking frame %d)\n",
                    final_pred->pred_error_m, final_pred->tracking_frame);

        // Frames to reach error thresholds
        auto frames_to_threshold = [&](double thresh) -> int {
            for (const auto& r : records)
                if (r.pred_valid && r.pred_error_m <= thresh)
                    return r.tracking_frame;
            return -1;
        };

        int f10 = frames_to_threshold(0.10);
        int f5  = frames_to_threshold(0.05);
        if (f10 >= 0) std::printf("  Frames to reach <10 cm: %d\n", f10);
        else          std::puts(  "  Frames to reach <10 cm: never");
        if (f5 >= 0)  std::printf("  Frames to reach <5 cm:  %d\n", f5);
        else          std::puts(  "  Frames to reach <5 cm:  never");

        // ETA accuracy
        std::vector<double> eta_errors;
        for (const auto& r : records)
            if (r.pred_valid) eta_errors.push_back(std::abs(r.eta_error_s));
        if (!eta_errors.empty()) {
            double eta_mean = std::accumulate(eta_errors.begin(), eta_errors.end(), 0.0) / eta_errors.size();
            double eta_max  = *std::max_element(eta_errors.begin(), eta_errors.end());
            std::printf("\n  ETA mean |error|:      %.3f s\n", eta_mean);
            std::printf("  ETA max |error|:       %.3f s\n", eta_max);
            std::printf("  ETA first prediction:  %+.3f s  (tracking frame %d)\n",
                        first_pred->eta_error_s, first_pred->tracking_frame);
            std::printf("  ETA final prediction:  %+.3f s  (tracking frame %d)\n",
                        final_pred->eta_error_s, final_pred->tracking_frame);
        }
    } else {
        std::puts("  No valid landing predictions generated.");
    }

    std::printf("\nTiming: %s\n", timing.summary().c_str());

    return 0;
}
