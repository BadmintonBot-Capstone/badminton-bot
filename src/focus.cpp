// live focus aid for every connected camera: full view, a zoomed crop and a sharpness score to turn the lens ring against
//   ./focus [--config config/] [--exposure us] [--gain db] [--fps n]
//   click a full view to move that camera's zoom box, + and - change the zoom, r resets the peaks, q quits

#include "core/config.hpp"
#include "single_cam.hpp"
#include <Spinnaker.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

namespace {

constexpr int PANE_W = 640, PANE_H = 400, ZOOM = 400;

struct View {
    Spinnaker::CameraPtr cam;
    std::string serial;
    int w = 0, h = 0;
    cv::Point centre;          // zoom box centre in full resolution pixels
    double score = 0, peak = 0;
    cv::Mat bgr;
};

struct Ui {
    std::vector<View>* views;
    int crop = 200;            // full resolution pixels across the zoom box
};

void on_mouse(int event, int x, int y, int, void* data)
{
    auto* ui = static_cast<Ui*>(data);
    int row = y / PANE_H;
    if (event != cv::EVENT_LBUTTONDOWN || x >= PANE_W || row >= int(ui->views->size())) return;
    auto& v = (*ui->views)[row];
    v.centre = {x * v.w / PANE_W, (y % PANE_H) * v.h / PANE_H};
    v.peak = v.score = 0;
}

cv::Rect zoom_box(const View& v, int crop)
{
    int x = std::clamp(v.centre.x - crop / 2, 0, v.w - crop);
    int y = std::clamp(v.centre.y - crop / 2, 0, v.h - crop);
    return {x, y, crop, crop};
}

} // namespace

int main(int argc, char** argv)
{
    std::string config_dir = "config";
    double exposure = 8333, gain = 6;   // the lab tracker settings, 8.33 ms also cancels the 120 Hz flicker
    int fps = 30;
    for (int i = 1; i < argc; ++i) {
        auto is = [&](const char* flag) { return !std::strcmp(argv[i], flag) && i + 1 < argc; };
        if      (is("--config"))   config_dir = argv[++i];
        else if (is("--exposure")) exposure   = std::atof(argv[++i]);
        else if (is("--gain"))     gain       = std::atof(argv[++i]);
        else if (is("--fps"))      fps        = std::atoi(argv[++i]);
    }

    auto cfg = baddy::load_config(config_dir);
    cfg.camera.exposure_us  = exposure;
    cfg.camera.gain_db      = gain;
    cfg.camera.fps          = fps;
    cfg.camera.pixel_format = "BayerRG8";

    auto system   = Spinnaker::System::GetInstance();
    auto cam_list = system->GetCameras();
    if (cam_list.GetSize() == 0) {
        std::puts("No cameras found. Is SpinView still open?");
        cam_list.Clear();
        system->ReleaseInstance();
        return 1;
    }

    std::vector<View> views(cam_list.GetSize());
    for (unsigned i = 0; i < cam_list.GetSize(); ++i) {
        auto& v = views[i];
        v.cam = cam_list.GetByIndex(i);
        v.cam->Init();
        auto applied = single_cam::configure(v.cam, cfg.camera);
        // a live view wants the newest frame, never a queue of old ones
        single_cam::set_enum(v.cam->GetTLStreamNodeMap(), "StreamBufferHandlingMode", "NewestOnly");
        v.serial = applied.serial;
        v.w = applied.width;
        v.h = applied.height;
        v.centre = {v.w / 2, v.h / 2};
        v.cam->BeginAcquisition();
    }
    cam_list.Clear();

    Ui ui{&views};
    const char* win = "focus";
    cv::namedWindow(win, cv::WINDOW_AUTOSIZE);
    cv::setMouseCallback(win, on_mouse, &ui);
    std::puts("[focus] click a full view to move its zoom box, + and - zoom, r resets peaks, q quits");

    cv::Mat canvas(PANE_H * int(views.size()), PANE_W + ZOOM, CV_8UC3);
    while (true) {
        for (auto& v : views) {
            Spinnaker::ImagePtr img = v.cam->GetNextImage(1000);
            if (!img->IsIncomplete()) {
                cv::Mat bayer(int(img->GetHeight()), int(img->GetWidth()), CV_8UC1, img->GetData());
                cv::cvtColor(bayer, v.bgr, cv::COLOR_BayerBG2BGR);
            }
            img->Release();
            if (v.bgr.empty()) continue;

            // sharpness is the variance of the laplacian inside the zoom box, smoothed over a few frames
            cv::Mat grey, lap;
            cv::cvtColor(v.bgr(zoom_box(v, ui.crop)), grey, cv::COLOR_BGR2GRAY);
            cv::Laplacian(grey, lap, CV_64F);
            cv::Scalar mean, sd;
            cv::meanStdDev(lap, mean, sd);
            v.score = v.score == 0 ? sd[0] * sd[0] : 0.7 * v.score + 0.3 * sd[0] * sd[0];
            v.peak  = std::max(v.peak, v.score);
        }

        canvas.setTo(cv::Scalar::all(0));
        for (int i = 0; i < int(views.size()); ++i) {
            auto& v = views[i];
            if (v.bgr.empty()) continue;
            cv::Mat bright;
            v.bgr.convertTo(bright, -1, 1.8);   // display only, the score uses the raw image

            cv::Mat full = canvas(cv::Rect(0, i * PANE_H, PANE_W, PANE_H));
            cv::resize(bright, full, full.size(), 0, 0, cv::INTER_AREA);
            cv::Rect box = zoom_box(v, ui.crop);
            double sx = double(PANE_W) / v.w, sy = double(PANE_H) / v.h;
            cv::rectangle(full, cv::Rect(int(box.x * sx), int(box.y * sy), int(box.width * sx), int(box.height * sy)),
                          {0, 255, 255}, 2);

            cv::Mat zoom = canvas(cv::Rect(PANE_W, i * PANE_H, ZOOM, ZOOM));
            cv::resize(bright(box), zoom, zoom.size(), 0, 0, cv::INTER_NEAREST);

            // the bar fills to the best score seen, so turn the ring until it is full again
            double frac = v.peak > 0 ? v.score / v.peak : 0;
            cv::rectangle(full, cv::Rect(10, PANE_H - 30, int((PANE_W - 20) * frac), 18),
                          frac > 0.97 ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 165, 255), cv::FILLED);
            char txt[128];
            std::snprintf(txt, sizeof(txt), "%s  sharp %.0f  peak %.0f  zoom %dpx", v.serial.c_str(), v.score, v.peak, ui.crop);
            cv::putText(full, txt, {10, 28}, cv::FONT_HERSHEY_SIMPLEX, 0.7, {0, 0, 0}, 4);
            cv::putText(full, txt, {10, 28}, cv::FONT_HERSHEY_SIMPLEX, 0.7, {255, 255, 255}, 2);
        }
        cv::imshow(win, canvas);

        int key = cv::waitKey(1);
        if (key == 'q') break;
        if (key == 'r') for (auto& v : views) v.peak = v.score = 0;
        if (key == '+' || key == '=') ui.crop = std::max(50, ui.crop / 2);
        if (key == '-') ui.crop = std::min(800, ui.crop * 2);
        if (key == '+' || key == '=' || key == '-') for (auto& v : views) v.peak = v.score = 0;
    }

    for (auto& v : views) {
        v.cam->EndAcquisition();
        v.cam->DeInit();
    }
    views.clear();
    system->ReleaseInstance();
    cv::destroyAllWindows();
}
