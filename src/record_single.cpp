// records raw bayer frames from one free-running camera into ram, then writes them to disk
//
// usage:
//   ./record_single --out <dir> [--config config/] [--exposure us] [--gain db] [--fps n]
//                   [--seconds s] [--delay s]
//
// writes <dir>/frames.raw (frames back to back), <dir>/frames.csv and <dir>/meta.yaml

#include "single_cam.hpp"
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <thread>
#include <vector>

int main(int argc, char** argv)
{
    std::string config_dir = "config", out;
    double exposure = -1, gain = -1, seconds = 5, delay = 0;
    int fps = -1;

    for (int i = 1; i < argc; ++i) {
        auto is = [&](const char* flag) { return !std::strcmp(argv[i], flag) && i + 1 < argc; };
        if      (is("--config"))   config_dir = argv[++i];
        else if (is("--out"))      out        = argv[++i];
        else if (is("--exposure")) exposure   = std::atof(argv[++i]);
        else if (is("--gain"))     gain       = std::atof(argv[++i]);
        else if (is("--fps"))      fps        = std::atoi(argv[++i]);
        else if (is("--seconds"))  seconds    = std::atof(argv[++i]);
        else if (is("--delay"))    delay      = std::atof(argv[++i]);
    }
    if (out.empty()) {
        std::puts("usage: record_single --out <dir> [--exposure us] [--gain db] [--fps n] [--seconds s] [--delay s]");
        return 1;
    }

    auto cfg = baddy::load_config(config_dir);
    if (exposure > 0) cfg.camera.exposure_us = exposure;
    if (gain >= 0)    cfg.camera.gain_db     = gain;
    if (fps > 0)      cfg.camera.fps         = fps;
    cfg.camera.pixel_format = "BayerRG8";

    auto system   = Spinnaker::System::GetInstance();
    auto cam_list = system->GetCameras();
    if (cam_list.GetSize() == 0) {
        std::puts("No cameras found.");
        cam_list.Clear();
        system->ReleaseInstance();
        return 1;
    }

    auto cam = cam_list.GetByIndex(0);
    cam->Init();
    auto applied = single_cam::configure(cam, cfg.camera);

    const size_t frame_bytes = size_t(applied.width) * applied.height;
    const int    n           = std::max(1, int(std::lround(seconds * applied.fps)));
    std::vector<uint8_t>  buf(frame_bytes * n);
    std::vector<uint64_t> ids(n), stamps(n);

    for (int s = int(delay); s > 0; --s) {
        std::printf("[record] starting in %d\n", s);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    std::printf("[record] recording %d frames (%.1f s)\n", n, n / applied.fps);
    cam->BeginAcquisition();

    int got = 0, incomplete = 0, errors = 0;
    while (got < n) {
        Spinnaker::ImagePtr img;
        try {
            img = cam->GetNextImage(1000);
        } catch (const Spinnaker::Exception& e) {
            std::fprintf(stderr, "[record] grab error: %s\n", e.what());
            if (++errors > 20) break;
            continue;
        }
        if (img->IsIncomplete() || img->GetImageSize() < frame_bytes) {
            ++incomplete;
            img->Release();
            continue;
        }
        std::memcpy(buf.data() + size_t(got) * frame_bytes, img->GetData(), frame_bytes);
        ids[got]    = img->GetFrameID();
        stamps[got] = img->GetTimeStamp();
        img->Release();
        ++got;
    }

    cam->EndAcquisition();
    cam->DeInit();
    cam = nullptr;
    cam_list.Clear();
    system->ReleaseInstance();

    std::filesystem::create_directories(out);
    std::ofstream(out + "/frames.raw", std::ios::binary)
        .write(reinterpret_cast<const char*>(buf.data()), std::streamsize(frame_bytes * got));

    std::ofstream csv(out + "/frames.csv");
    csv << "index,frame_id,timestamp_ns\n";
    for (int i = 0; i < got; ++i) csv << i << ',' << ids[i] << ',' << stamps[i] << '\n';

    const long dropped = got > 1 ? long(ids[got - 1] - ids[0] + 1) - got : 0;
    const double span_s = got > 1 ? double(stamps[got - 1] - stamps[0]) * 1e-9 : 0.0;

    std::ofstream meta(out + "/meta.yaml");
    meta << "serial: \"" << applied.serial << "\"\n"
         << "width: " << applied.width << "\nheight: " << applied.height << '\n'
         << "pixel_format: BayerRG8\n"
         << "exposure_us: " << applied.exposure_us << "\ngain_db: " << applied.gain_db << '\n'
         << "fps: " << applied.fps << "\nframes: " << got << '\n'
         << "dropped: " << dropped << "\nincomplete: " << incomplete << '\n';

    std::printf("[record] %d frames, %ld dropped, %d incomplete, measured %.1f fps, saved to %s\n",
                got, dropped, incomplete, span_s > 0 ? (got - 1) / span_s : 0.0, out.c_str());
    return 0;
}
