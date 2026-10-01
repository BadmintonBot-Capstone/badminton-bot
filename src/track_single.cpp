// live or replayed single camera shuttle tracking with the motion detector
//
// usage:
//   ./track_single [--config config/] [--exposure us] [--gain db] [--fps n] [--headless]
//   ./track_single --replay <recording dir> [--camera left|right] [--out track.csv] [--headless]
//
// keys: q quit, h display on/off

#include "shuttle_tracker.hpp"
#include "single_cam.hpp"
#include "core/timing.hpp"
#include <opencv2/highgui.hpp>
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <memory>
#include <vector>

// rolling mean and p95 of the last few hundred samples
struct Stats {
    std::vector<double> v;
    size_t next = 0, cap = 500;
    void add(double x)
    {
        if (v.size() < cap) v.push_back(x);
        else v[next++ % cap] = x;
    }
    double mean() const
    {
        double s = 0;
        for (double x : v) s += x;
        return v.empty() ? 0 : s / v.size();
    }
    double p95() const
    {
        if (v.empty()) return 0;
        auto s = v;
        std::nth_element(s.begin(), s.begin() + s.size() * 95 / 100, s.end());
        return s[s.size() * 95 / 100];
    }
};

// plays back one camera of a record_stereo clip (left.raw, right.raw) or a record_single clip (frames.raw)
struct Replay {
    std::ifstream raw;
    int w = 0, h = 0, n = 0, i = 0;
    cv::Mat frame;

    Replay(const std::string& dir, const std::string& camera)
    {
        YAML::Node m = YAML::LoadFile(dir + "/meta.yaml");
        w = m["width"].as<int>();
        h = m["height"].as<int>();
        n = m["frames"].as<int>();
        std::string path = dir + "/" + camera + ".raw";
        if (!std::filesystem::exists(path)) path = dir + "/frames.raw";
        raw.open(path, std::ios::binary);
        frame.create(h, w, CV_8UC1);
    }
    bool next()
    {
        if (i >= n || !raw.read(reinterpret_cast<char*>(frame.data), std::streamsize(w) * h)) return false;
        ++i;
        return true;
    }
};

static void draw(cv::Mat& disp, const shuttle::Detector& det, const std::vector<shuttle::Candidate>& cands,
                 const shuttle::Output& out, double fps, double det_ms)
{
    cv::cvtColor(det.grey(), disp, cv::COLOR_GRAY2BGR);
    for (auto& r : det.people()) cv::rectangle(disp, r, {120, 120, 120}, 1);
    for (auto& c : cands) cv::circle(disp, c.c, 4, {0, 200, 255}, 1);
    if (out.track) {
        auto& tr = out.track->trail;
        for (size_t k = 1; k < tr.size(); ++k) cv::line(disp, tr[k - 1], tr[k], {0, 0, 255}, 2);
        cv::Scalar col = out.state == shuttle::State::Hit ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 140, 255);
        cv::circle(disp, out.p, 12, col, 2);
    }
    char txt[96];
    std::snprintf(txt, sizeof(txt), "%.0f fps  detect %.1f ms  %s", fps, det_ms,
                  out.track ? (out.state == shuttle::State::Hit ? "TRACKING" : "COAST") : "searching");
    cv::putText(disp, txt, {10, 28}, cv::FONT_HERSHEY_SIMPLEX, 0.7, {0, 255, 0}, 2);
}

int main(int argc, char** argv)
{
    std::string config_dir = "config", replay_dir, out_csv, replay_cam = "left";
    double exposure = -1, gain = -1;
    int fps = -1;
    bool show = true;

    for (int i = 1; i < argc; ++i) {
        auto is = [&](const char* flag) { return !std::strcmp(argv[i], flag) && i + 1 < argc; };
        if      (is("--config"))   config_dir = argv[++i];
        else if (is("--replay"))   replay_dir = argv[++i];
        else if (is("--camera"))   replay_cam = argv[++i];
        else if (is("--out"))      out_csv    = argv[++i];
        else if (is("--exposure")) exposure   = std::atof(argv[++i]);
        else if (is("--gain"))     gain       = std::atof(argv[++i]);
        else if (is("--fps"))      fps        = std::atoi(argv[++i]);
        else if (!std::strcmp(argv[i], "--headless")) show = false;
    }

    auto params = shuttle::load_params(config_dir + "/motion.yaml");

    // frame source, either a recording or the first camera found
    std::unique_ptr<Replay> replay;
    Spinnaker::SystemPtr system;
    Spinnaker::CameraPtr cam;
    int w = 0, h = 0;
    if (!replay_dir.empty()) {
        replay = std::make_unique<Replay>(replay_dir, replay_cam);
        w = replay->w;
        h = replay->h;
        std::printf("[track] replaying %d frames from %s\n", replay->n, replay_dir.c_str());
    } else {
        auto cfg = baddy::load_config(config_dir);
        if (exposure > 0) cfg.camera.exposure_us = exposure;
        if (gain >= 0)    cfg.camera.gain_db     = gain;
        if (fps > 0)      cfg.camera.fps         = fps;
        cfg.camera.pixel_format = "BayerRG8";

        system = Spinnaker::System::GetInstance();
        auto cam_list = system->GetCameras();
        if (cam_list.GetSize() == 0) {
            std::puts("No cameras found.");
            cam_list.Clear();
            system->ReleaseInstance();
            return 1;
        }
        cam = cam_list.GetByIndex(0);
        cam_list.Clear();
        cam->Init();
        auto applied = single_cam::configure(cam, cfg.camera);
        w = applied.width;
        h = applied.height;
        cam->BeginAcquisition();
    }

    shuttle::Detector det(params);
    shuttle::Tracker  trk(params, w / 2, h / 2);

    std::ofstream csv;
    if (!out_csv.empty()) {
        csv.open(out_csv);
        csv << "frame,state,x,y,detect_ms\n";
    }
    if (show) cv::namedWindow("track_single", cv::WINDOW_NORMAL);

    cv::Mat live, disp;
    Stats det_log;
    int frame = 0, found = 0, dropped = 0;
    uint64_t last_id = 0;
    double frame_us = 0;

    while (true) {
        baddy::ScopedTimer frame_timer(&frame_us);
        const cv::Mat* bayer = nullptr;
        Spinnaker::ImagePtr img;

        if (replay) {
            if (!replay->next()) break;
            bayer = &replay->frame;
        } else {
            try {
                img = cam->GetNextImage(1000);
            } catch (const Spinnaker::Exception& e) {
                std::fprintf(stderr, "[camera] grab error: %s\n", e.what());
                continue;
            }
            if (img->IsIncomplete()) {
                img->Release();
                continue;
            }
            uint64_t id = img->GetFrameID();
            if (frame > 0 && id > last_id + 1) dropped += int(id - last_id - 1);
            last_id = id;
            live = cv::Mat(h, w, CV_8UC1, img->GetData());
            bayer = &live;
        }

        double det_us = 0;
        shuttle::Output out;
        const std::vector<shuttle::Candidate>* cands;
        {
            baddy::ScopedTimer t(&det_us);
            cands = &det.step(*bayer);
            out   = trk.step(*cands);
        }
        det_log.add(det_us);
        if (img) img->Release();

        if (out.track) ++found;
        if (csv.is_open()) {
            const char* st = !out.track ? "search" : out.state == shuttle::State::Hit ? "hit" : "coast";
            csv << frame << ',' << st << ',';
            if (out.track) csv << 2 * out.p.x << ',' << 2 * out.p.y;
            else           csv << ',';
            csv << ',' << det_us / 1000 << '\n';
        }

        int key = -1;
        if (show && frame % 3 == 0) {
            draw(disp, det, *cands, out, frame_us > 0 ? 1e6 / frame_us : 0, det_us / 1000);
            cv::imshow("track_single", disp);
            key = cv::waitKey(1) & 0xFF;
        }
        if (key == 'q') break;
        if (key == 'h') {
            show = !show;
            if (!show) cv::destroyAllWindows();
        }

        if (!replay && frame % 100 == 0)
            std::printf("[track] frame %d  detect mean %.2f ms p95 %.2f ms  dropped %d  %s\n", frame,
                        det_log.mean() / 1000, det_log.p95() / 1000, dropped,
                        out.track ? "TRACKING" : "searching");
        ++frame;
    }

    if (cam) {
        cam->EndAcquisition();
        cam->DeInit();
        cam = nullptr;
        system->ReleaseInstance();
    }
    std::printf("[track] %d frames, shuttle reported in %d, detect mean %.2f ms p95 %.2f ms, dropped %d\n",
                frame, found, det_log.mean() / 1000, det_log.p95() / 1000, dropped);
    return 0;
}
