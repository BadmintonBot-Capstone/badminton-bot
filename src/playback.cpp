// playback.cpp — plays back a recording made by preview (primary.raw, timestamps.csv, metadata.txt)
// or by record_single (frames.raw, frames.csv, meta.yaml).
//
// usage: playback <recording_dir> [speed]
//
// Plays at the recorded rate using the camera timestamps (speed 0.5 = half speed).
// Frames lost while recording (gaps in id_primary) are reported in the overlay and the console.
// A stereo recording (cameras=2 in metadata.txt, with secondary.raw) is shown side by side,
// primary on the left, each camera at half size; a pair whose frame counters have moved apart
// is flagged in the overlay.
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
    int64_t id_s = -1;   // secondary frame counter, -1 in a single-camera recording
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

// record_single's meta.yaml: flat "key: value" lines, strings quoted.
static std::map<std::string, std::string> read_meta_yaml(const std::string& path)
{
    std::map<std::string, std::string> m;
    std::ifstream f(path);
    std::string line;
    while (std::getline(f, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        const auto colon = line.find(':');
        if (colon == std::string::npos) continue;
        std::string v = line.substr(colon + 1);
        v.erase(0, v.find_first_not_of(' '));
        if (v.size() >= 2 && v.front() == '"' && v.back() == '"') v = v.substr(1, v.size() - 2);
        m[line.substr(0, colon)] = v;
    }
    return m;
}

// record_single's frames.csv: index,frame_id,timestamp_ns
static std::vector<Row> read_frames_csv(const std::string& path)
{
    std::vector<Row> rows;
    std::ifstream f(path);
    std::string line;
    std::getline(f, line);  // header
    while (std::getline(f, line)) {
        std::vector<std::string> c;
        std::stringstream ss(line);
        std::string cell;
        while (std::getline(ss, cell, ',')) c.push_back(cell);
        if (c.size() < 3) continue;
        rows.push_back({std::stoll(c[2]), std::stoll(c[1]), -1});
    }
    return rows;
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
        const bool has_s = c.size() > 4 && !c[4].empty();
        rows.push_back({std::stoll(c[1]), std::stoll(c[3]), has_s ? std::stoll(c[4]) : -1});
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

    // A preview recording has metadata.txt; otherwise try record_single's layout
    // (meta.yaml, frames.raw, frames.csv), which is always one camera.
    auto meta = read_metadata(dir + "/metadata.txt");
    const bool single = meta.empty();
    if (single) meta = read_meta_yaml(dir + "/meta.yaml");
    if (meta.empty()) {
        std::fprintf(stderr, "Cannot read %s/metadata.txt or meta.yaml\n", dir.c_str());
        return 1;
    }
    const int width = std::stoi(meta["width"]);
    const int height = std::stoi(meta["height"]);
    if (meta["pixel_format"] != "BayerRG8") {
        std::fprintf(stderr, "Only BayerRG8 is supported, not %s\n", meta["pixel_format"].c_str());
        return 1;
    }
    const int64_t bytes_per_image =
        single ? static_cast<int64_t>(width) * height : std::stoll(meta["bytes_per_image"]);

    const auto rows = single ? read_frames_csv(dir + "/frames.csv") : read_timestamps(dir + "/timestamps.csv");
    std::ifstream raw(dir + (single ? "/frames.raw" : "/primary.raw"), std::ios::binary);
    if (!raw || rows.empty()) {
        std::fprintf(stderr, "Cannot read %s in %s\n",
                     single ? "frames.raw or frames.csv" : "primary.raw or timestamps.csv", dir.c_str());
        return 1;
    }

    // Stereo: the secondary camera's images, same size and order as primary.raw.
    std::ifstream raw_s;
    const bool stereo = meta["cameras"] == "2";
    if (stereo) {
        raw_s.open(dir + "/secondary.raw", std::ios::binary);
        if (!raw_s || meta["secondary_bytes_per_image"] != meta["bytes_per_image"]) {
            std::fprintf(stderr, "Stereo recording, but secondary.raw is missing or a different size\n");
            return 1;
        }
    }

    // Frames lost during recording: missing[i] is how many came just before frame i.
    std::vector<int64_t> missing(rows.size(), 0);
    int64_t total_missing = 0;
    for (size_t i = 1; i < rows.size(); ++i) {
        missing[i] = std::max<int64_t>(0, rows[i].id - rows[i - 1].id - 1);
        total_missing += missing[i];
    }
    // Stereo pairs whose counters have moved apart since the first pair (preview drops these,
    // so any here point at a recording problem).
    std::vector<char> out_of_step(rows.size(), 0);
    int64_t total_out_of_step = 0;
    if (stereo) {
        for (size_t i = 0; i < rows.size(); ++i) {
            out_of_step[i] = (rows[i].id - rows[0].id) != (rows[i].id_s - rows[0].id_s);
            total_out_of_step += out_of_step[i];
        }
    }
    const double duration_s = (rows.back().ts - rows.front().ts) / 1e9;
    std::printf("%zu %s, %.2f s, %dx%d, %lld %s missing\n", rows.size(), stereo ? "pairs" : "frames",
                duration_s, width, height, static_cast<long long>(total_missing), stereo ? "pairs" : "frames");
    if (stereo && total_out_of_step)
        std::printf("%lld pairs have frame counters out of step\n", static_cast<long long>(total_out_of_step));
    std::puts("space pause, a/d step, j/l jump 100 frames, +/- speed, q quit");

    const std::string win = "playback: " + dir;
    cv::namedWindow(win, cv::WINDOW_NORMAL);
    cv::resizeWindow(win, stereo ? 1920 : 960, 600);

    cv::Mat bayer(height, width, CV_8UC1), bgr;
    cv::Mat bayer_s(height, width, CV_8UC1), bgr_s, half_p, half_s;
    // Overlay text is drawn on the shown image, which is half size in stereo.
    const double text_scale = stereo ? 0.7 : 1.4;
    const int text_dy = stereo ? 30 : 60, text_thick = stereo ? 1 : 2;
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
        if (stereo) {
            raw_s.seekg(i * bytes_per_image);
            raw_s.read(reinterpret_cast<char*>(bayer_s.data), bytes_per_image);
            if (!raw_s) {
                std::fprintf(stderr, "secondary.raw ended early at frame %lld\n", static_cast<long long>(i));
                break;
            }
            cv::cvtColor(bayer_s, bgr_s, cv::COLOR_BayerBG2BGR);
            // Side by side at half size each, so the pair fits a 1920 wide window.
            cv::resize(bgr, half_p, {}, 0.5, 0.5, cv::INTER_AREA);
            cv::resize(bgr_s, half_s, {}, 0.5, 0.5, cv::INTER_AREA);
            cv::hconcat(half_p, half_s, bgr);
            cv::line(bgr, {width / 2, 0}, {width / 2, height / 2}, {0, 0, 0}, 2);
        }

        auto label = [&](const char* s, int line, const cv::Scalar& colour) {
            const cv::Point at{20, text_dy * (line + 1) - 10};
            cv::putText(bgr, s, at, cv::FONT_HERSHEY_SIMPLEX, text_scale, {0, 0, 0}, text_thick + 4);
            cv::putText(bgr, s, at, cv::FONT_HERSHEY_SIMPLEX, text_scale, colour, text_thick);
        };
        if (missing[i] > 0) gap_shown_until = i + 25;
        const double t = (rows[i].ts - rows[0].ts) / 1e9;
        char text[200];
        std::snprintf(text, sizeof text, "%s %lld/%lld  t=%.2f s  speed %.2gx%s%s",
                      stereo ? "pair" : "frame", static_cast<long long>(i), static_cast<long long>(n - 1),
                      t, speed, stereo ? "   primary | secondary" : "", paused ? "  PAUSED" : "");
        label(text, 0, {255, 255, 255});
        int line = 1;
        if (i <= gap_shown_until) {
            int64_t near = 0;
            for (int64_t k = std::max<int64_t>(1, i - 25); k <= i; ++k) near += missing[k];
            std::snprintf(text, sizeof text, "FRAMES MISSING: %lld in last 25", static_cast<long long>(near));
            label(text, line++, {0, 0, 255});
        }
        if (out_of_step[i]) {
            std::snprintf(text, sizeof text, "PAIR OUT OF STEP: primary %lld, secondary %lld",
                          static_cast<long long>(rows[i].id), static_cast<long long>(rows[i].id_s));
            label(text, line++, {0, 0, 255});
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
