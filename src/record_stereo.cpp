// records raw bayer pairs from both cameras on the hardware trigger into ram, then writes them to disk
//
// usage:
//   systemd-inhibit --what=sleep:idle:handle-lid-switch \
//   ./record_stereo --out <dir> [--config config/] [--exposure us] [--gain db] [--fps n]
//                   [--seconds s] [--delay s] [--note key=value ...] [--headless]
//
// writes <dir>/left.raw and <dir>/right.raw (frames back to back), <dir>/frames.csv and <dir>/meta.yaml.
// the window shows a live view and a big countdown, then the seconds left, so one person can run it.

#include "core/camera.hpp"
#include "core/config.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <string>
#include <utility>
#include <vector>

namespace {

// free memory the kernel says it can hand out, in bytes
size_t mem_available()
{
    std::ifstream f("/proc/meminfo");
    std::string key;
    size_t kb = 0;
    while (f >> key >> kb) {
        if (key == "MemAvailable:") return kb * 1024;
        f.ignore(256, '\n');
    }
    return 0;
}

std::string run(const std::string& cmd)
{
    std::array<char, 256> line{};
    std::string out;
    if (FILE* p = popen(cmd.c_str(), "r")) {
        while (fgets(line.data(), int(line.size()), p)) out += line.data();
        pclose(p);
    }
    while (!out.empty() && (out.back() == '\n' || out.back() == '\r')) out.pop_back();
    return out;
}

void big_text(cv::Mat& img, const std::string& text, cv::Scalar col)
{
    int base = 0;
    cv::Size sz = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 5, 14, &base);
    cv::Point org((img.cols - sz.width) / 2, (img.rows + sz.height) / 2);
    cv::putText(img, text, org, cv::FONT_HERSHEY_SIMPLEX, 5, {0, 0, 0}, 28);
    cv::putText(img, text, org, cv::FONT_HERSHEY_SIMPLEX, 5, col, 14);
}

} // namespace

int main(int argc, char** argv)
{
    std::string config_dir = "config", out;
    double exposure = -1, gain = -1, seconds = 30, delay = 10;
    int fps = -1;
    bool show = true;
    std::vector<std::pair<std::string, std::string>> notes;

    for (int i = 1; i < argc; ++i) {
        auto is = [&](const char* flag) { return !std::strcmp(argv[i], flag) && i + 1 < argc; };
        if      (is("--config"))   config_dir = argv[++i];
        else if (is("--out"))      out        = argv[++i];
        else if (is("--exposure")) exposure   = std::atof(argv[++i]);
        else if (is("--gain"))     gain       = std::atof(argv[++i]);
        else if (is("--fps"))      fps        = std::atoi(argv[++i]);
        else if (is("--seconds"))  seconds    = std::atof(argv[++i]);
        else if (is("--delay"))    delay      = std::atof(argv[++i]);
        else if (is("--note")) {
            std::string kv = argv[++i];
            auto eq = kv.find('=');
            notes.emplace_back(kv.substr(0, eq), eq == std::string::npos ? "" : kv.substr(eq + 1));
        }
        else if (!std::strcmp(argv[i], "--headless")) show = false;
    }
    if (out.empty()) {
        std::puts("usage: record_stereo --out <dir> [--exposure us] [--gain db] [--fps n] [--seconds s] "
                  "[--delay s] [--note key=value ...] [--headless]");
        return 1;
    }
    if (std::filesystem::exists(out + "/left.raw")) {
        std::printf("[record] %s already has a clip, pick a new --out\n", out.c_str());
        return 1;
    }

    auto cfg = baddy::load_config(config_dir);
    if (exposure > 0) cfg.camera.exposure_us = exposure;
    if (gain >= 0)    cfg.camera.gain_db     = gain;
    if (fps > 0)      cfg.camera.fps         = fps;
    cfg.camera.pixel_format = "BayerRG8";

    baddy::StereoCamera camera(cfg.camera);
    camera.deep_buffers(120);
    const auto ap = camera.applied(true), as = camera.applied(false);
    std::printf("[record] left %s and right %s: %dx%d %s  exposure %.0f us  gain %.1f dB  %.1f fps\n",
                ap.serial.c_str(), as.serial.c_str(), ap.width, ap.height, ap.pixel_format.c_str(),
                ap.exposure_us, ap.gain_db, ap.fps);

    // every frame goes into memory that is allocated and touched before recording starts
    const size_t fb = size_t(ap.width) * ap.height;
    const int    n  = std::max(1, int(std::lround(seconds * ap.fps)));
    const size_t need = 2 * fb * size_t(n);
    if (need > mem_available() * 0.8) {
        std::printf("[record] %.1f GB needed but only %.1f GB free, shorten --seconds\n",
                    need / 1e9, mem_available() / 1e9);
        return 1;
    }
    std::vector<uint8_t> buf_l(fb * n), buf_r(fb * n);
    std::vector<uint64_t> id_l(n), id_r(n);
    std::vector<int64_t>  ts_l(n), ts_r(n);

    const char* win = "record_stereo";
    if (show) {
        cv::namedWindow(win, cv::WINDOW_NORMAL);
        cv::resizeWindow(win, 960, 600);
    }
    cv::Mat thumb, disp;
    auto draw = [&](const cv::Mat& bayer, const std::string& text, cv::Scalar col) {
        if (!show) return;
        cv::resize(bayer, thumb, {960, 600}, 0, 0, cv::INTER_AREA);
        cv::cvtColor(thumb, disp, cv::COLOR_GRAY2BGR);
        big_text(disp, text, col);
        cv::imshow(win, disp);
        cv::waitKey(1);
    };

    camera.start();
    using clock = std::chrono::steady_clock;

    // countdown with a live view so the shot can be framed, frames here are thrown away
    baddy::StereoCamera::RawPair live;
    const auto go = clock::now() + std::chrono::milliseconds(int(delay * 1000));
    int last_shown = -1;
    for (int k = 0; clock::now() < go; ++k) {
        if (!camera.grab_raw(live)) continue;
        int left_s = int(std::ceil(std::chrono::duration<double>(go - clock::now()).count()));
        if (k % 4 == 0) draw(live.left, std::to_string(left_s), {0, 255, 255});
        if (!show && left_s != last_shown) std::printf("[record] starting in %d\n", left_s);
        last_shown = left_s;
    }

    std::printf("[record] recording %d pairs (%.1f s)\n", n, n / ap.fps);
    int got = 0, failed = 0, in_a_row = 0;
    baddy::StereoCamera::RawPair pair;
    while (got < n) {
        pair.left  = cv::Mat(ap.height, ap.width, CV_8UC1, buf_l.data() + size_t(got) * fb);
        pair.right = cv::Mat(ap.height, ap.width, CV_8UC1, buf_r.data() + size_t(got) * fb);
        if (!camera.grab_raw(pair)) {
            ++failed;
            if (++in_a_row > 50) {
                std::puts("[record] 50 failed grabs in a row, stopping");
                break;
            }
            continue;
        }
        in_a_row = 0;
        id_l[got] = pair.left_id;
        id_r[got] = pair.right_id;
        ts_l[got] = pair.left_ts;
        ts_r[got] = pair.right_ts;
        ++got;
        if (got % 10 == 0) {
            double left_s = (n - got) / ap.fps;
            char txt[32];
            std::snprintf(txt, sizeof(txt), "REC %.0f", std::ceil(left_s));
            draw(pair.left, txt, {0, 0, 255});
        }
    }
    camera.stop();
    if (show) {
        disp.setTo(cv::Scalar::all(0));
        big_text(disp, "SAVING", {0, 255, 0});
        cv::imshow(win, disp);
        cv::waitKey(1);
    }

    // the two counters have to move together, otherwise a pair mixes two moments
    long mismatched = 0;
    for (int i = 1; i < got; ++i)
        if (id_l[i] - id_l[0] != id_r[i] - id_r[0]) ++mismatched;
    const long dropped_l = got > 1 ? long(id_l[got - 1] - id_l[0] + 1) - got : 0;
    const long dropped_r = got > 1 ? long(id_r[got - 1] - id_r[0] + 1) - got : 0;
    const double span_s  = got > 1 ? double(ts_l[got - 1] - ts_l[0]) * 1e-9 : 0.0;

    std::filesystem::create_directories(out);
    std::ofstream(out + "/left.raw", std::ios::binary)
        .write(reinterpret_cast<const char*>(buf_l.data()), std::streamsize(fb * got));
    std::ofstream(out + "/right.raw", std::ios::binary)
        .write(reinterpret_cast<const char*>(buf_r.data()), std::streamsize(fb * got));

    std::ofstream csv(out + "/frames.csv");
    csv << "index,left_id,right_id,left_ts_ns,right_ts_ns\n";
    for (int i = 0; i < got; ++i)
        csv << i << ',' << id_l[i] << ',' << id_r[i] << ',' << ts_l[i] << ',' << ts_r[i] << '\n';

    char when[32];
    std::time_t now = std::time(nullptr);
    std::strftime(when, sizeof(when), "%Y-%m-%dT%H:%M:%S", std::localtime(&now));
    std::ofstream meta(out + "/meta.yaml");
    meta << "recorded: \"" << when << "\"\n"
         << "git: \"" << run("git -C '" + config_dir + "' describe --always --dirty") << "\"\n"
         << "calibration: \"" << config_dir << "/calibration.yaml\"\n"
         << "width: " << ap.width << "\nheight: " << ap.height << '\n'
         << "pixel_format: " << ap.pixel_format << '\n'
         << "exposure_us: " << ap.exposure_us << "\ngain_db: " << ap.gain_db << '\n'
         << "fps: " << ap.fps << '\n'
         << "left:\n  serial: \"" << ap.serial << "\"\n  exposure_us: " << ap.exposure_us
         << "\n  gain_db: " << ap.gain_db << '\n'
         << "right:\n  serial: \"" << as.serial << "\"\n  exposure_us: " << as.exposure_us
         << "\n  gain_db: " << as.gain_db << '\n'
         << "frames: " << got << '\n'
         << "dropped_left: " << dropped_l << "\ndropped_right: " << dropped_r << '\n'
         << "mismatched_pairs: " << mismatched << "\nfailed_grabs: " << failed << '\n'
         << "notes:\n";
    for (auto& [k, v] : notes) meta << "  " << k << ": \"" << v << "\"\n";

    std::printf("[record] %d pairs, dropped %ld left %ld right, %ld mismatched, %d failed grabs, "
                "measured %.1f fps, saved to %s\n",
                got, dropped_l, dropped_r, mismatched, failed, span_s > 0 ? (got - 1) / span_s : 0.0, out.c_str());
    if (mismatched > 0)
        std::puts("[record] warning: left and right frame counters drifted apart, check the trigger cable");
    return 0;
}
