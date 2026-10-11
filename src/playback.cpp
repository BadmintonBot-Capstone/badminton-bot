// playback.cpp — plays back a recording made by preview (primary.raw, timestamps.csv, metadata.txt).
//
// usage: playback <recording_dir> [speed]
//
// Plays at the recorded rate using the camera timestamps (speed 0.5 = half speed).
// Frames lost while recording (gaps in id_primary) are reported in the overlay and the console.
//
// keys: space pause/resume, a/d step back/forward one frame (while paused),
//       j/l jump 1 s back/forward, +/- change speed, q or Esc quit.

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

struct Row {
    int64_t ts = 0;
    int64_t id = 0;
};

static std::map<std::string, std::string> read_metadata(const std::string& path)
{
    std::map<std::string, std::string> m;
    std::ifstream f(path);
    std::string line;
    while (std::getline(f, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        const auto eq = line.find('=');
        if (eq != std::string::npos) m[line.substr(0, eq)] = line.substr(eq + 1);
    }
    return m;
}

static std::vector<Row> read_timestamps(const std::string& path)
{
    // columns: index,ts_primary,ts_secondary,id_primary,id_secondary,offset_drift_ns
    std::vector<Row> rows;
    std::ifstream f(path);
    std::string line;
    std::getline(f, line);  // header
    while (std::getline(f, line)) {
        std::vector<std::string> c;
        std::stringstream ss(line);
        std::string cell;
        while (std::getline(ss, cell, ',')) c.push_back(cell);
        if (c.size() < 4) continue;
        rows.push_back({std::stoll(c[1]), std::stoll(c[3])});
    }
    return rows;
}

int main(int argc, char* argv[])
{
    if (argc < 2) {
        std::puts("usage: playback <recording_dir> [speed]");
        return 1;
    }
    const std::string dir = argv[1];
    double speed = argc > 2 ? std::stod(argv[2]) : 1.0;

    auto meta = read_metadata(dir + "/metadata.txt");
    if (meta.empty()) {
        std::fprintf(stderr, "Cannot read %s/metadata.txt\n", dir.c_str());
        return 1;
    }
    const int width = std::stoi(meta["width"]);
    const int height = std::stoi(meta["height"]);
    const int64_t bytes_per_image = std::stoll(meta["bytes_per_image"]);
    if (meta["pixel_format"] != "BayerRG8") {
        std::fprintf(stderr, "Only BayerRG8 is supported, not %s\n", meta["pixel_format"].c_str());
        return 1;
    }

    const auto rows = read_timestamps(dir + "/timestamps.csv");
    std::ifstream raw(dir + "/primary.raw", std::ios::binary);
    if (!raw || rows.empty()) {
        std::fprintf(stderr, "Cannot read primary.raw or timestamps.csv in %s\n", dir.c_str());
        return 1;
    }

    // Frames lost during recording: missing[i] is how many came just before frame i.
    std::vector<int64_t> missing(rows.size(), 0);
    int64_t total_missing = 0;
    for (size_t i = 1; i < rows.size(); ++i) {
        missing[i] = std::max<int64_t>(0, rows[i].id - rows[i - 1].id - 1);
        total_missing += missing[i];
    }
    const double duration_s = (rows.back().ts - rows.front().ts) / 1e9;
    std::printf("%zu frames, %.2f s, %dx%d, %lld frames missing\n",
                rows.size(), duration_s, width, height, static_cast<long long>(total_missing));
    std::puts("space pause, a/d step, j/l jump 1 s, +/- speed, q quit");

    const std::string win = "playback: " + dir;
    cv::namedWindow(win, cv::WINDOW_NORMAL);
    cv::resizeWindow(win, 960, 600);

    cv::Mat bayer(height, width, CV_8UC1), bgr;
    const int64_t n = static_cast<int64_t>(rows.size());
    int64_t i = 0;
    bool paused = false;
    int64_t gap_shown_until = -1;  // keep a gap warning visible for a short while

    using clock = std::chrono::steady_clock;
    auto wall_start = clock::now();
    int64_t ts_start = rows[0].ts;

    while (true) {
        raw.seekg(i * bytes_per_image);
        raw.read(reinterpret_cast<char*>(bayer.data), bytes_per_image);
        if (!raw) {
            std::fprintf(stderr, "primary.raw ended early at frame %lld\n", static_cast<long long>(i));
            break;
        }
        // Same conversion as analysis/rec.py: OpenCV names this sensor's RGGB layout "BG".
        cv::cvtColor(bayer, bgr, cv::COLOR_BayerBG2BGR);

        if (missing[i] > 0) gap_shown_until = i + 25;
        const double t = (rows[i].ts - rows[0].ts) / 1e9;
        char text[160];
        std::snprintf(text, sizeof text, "frame %lld/%lld  t=%.2f s  speed %.2gx%s",
                      static_cast<long long>(i), static_cast<long long>(n - 1), t, speed,
                      paused ? "  PAUSED" : "");
        cv::putText(bgr, text, {20, 50}, cv::FONT_HERSHEY_SIMPLEX, 1.4, {0, 0, 0}, 6);
        cv::putText(bgr, text, {20, 50}, cv::FONT_HERSHEY_SIMPLEX, 1.4, {255, 255, 255}, 2);
        if (i <= gap_shown_until) {
            int64_t near = 0;
            for (int64_t k = std::max<int64_t>(1, i - 25); k <= i; ++k) near += missing[k];
            std::snprintf(text, sizeof text, "FRAMES MISSING: %lld in last 25", static_cast<long long>(near));
            cv::putText(bgr, text, {20, 110}, cv::FONT_HERSHEY_SIMPLEX, 1.4, {0, 0, 0}, 6);
            cv::putText(bgr, text, {20, 110}, cv::FONT_HERSHEY_SIMPLEX, 1.4, {0, 0, 255}, 2);
        }
        cv::imshow(win, bgr);

        // Wait until this frame is due, polling keys meanwhile.
        int key = -1;
        if (paused) {
            key = cv::waitKey(0);
        } else {
            const auto due = wall_start + std::chrono::nanoseconds(
                static_cast<int64_t>((rows[i].ts - ts_start) / speed));
            do {
                key = cv::waitKey(1);
            } while (key < 0 && clock::now() < due);
        }
        if (cv::getWindowProperty(win, cv::WND_PROP_VISIBLE) < 1) break;

        bool resync = false;
        int64_t next = paused ? i : i + 1;
        switch (key) {
        case 'q': case 27: next = -1; break;
        case ' ': paused = !paused; resync = true; next = i + (paused ? 0 : 1); break;
        case 'a': next = i - 1; paused = true; break;
        case 'd': next = i + 1; paused = true; break;
        case 'j': next = i - 100; resync = true; break;
        case 'l': next = i + 100; resync = true; break;
        case '+': case '=': speed = std::min(speed * 2, 8.0); resync = true; break;
        case '-': case '_': speed = std::max(speed / 2, 0.0625); resync = true; break;
        default: break;
        }
        if (next < 0 && key != 'a' && key != 'j') break;
        if (next >= n) {
            // Hold on the last frame until the user quits or steps back.
            next = n - 1;
            paused = true;
        }
        i = std::clamp<int64_t>(next, 0, n - 1);
        if (resync) {
            wall_start = clock::now();
            ts_start = rows[i].ts;
        }
    }
    cv::destroyAllWindows();
    return 0;
}
