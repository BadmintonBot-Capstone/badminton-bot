// diff_detect_single.cpp — Frame differencing with a single free-running camera.
// No hardware triggers, no StereoCamera, no rectification.
// Uses Spinnaker directly like preview.cpp.
//
// Usage:
//   ./diff_detect_single [--config config/]
//
// Keys: 'q' quit, 'h' toggle display on/off

#include "core/config.hpp"
#include "core/timing.hpp"
#include <Spinnaker.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <cstdio>
#include <cstring>
#include <vector>

namespace GenApi = Spinnaker::GenApi;

static int diff_lo  = 30;
static int diff_hi  = 255;
static int min_area = 5;
static int max_area = 5000;

static constexpr int DISP_W = 640;
static constexpr int DISP_H = 400;

static void set_enum(GenApi::INodeMap& nm, const char* name, const char* value)
{
    GenApi::CEnumerationPtr node = nm.GetNode(name);
    if (node && GenApi::IsWritable(node))
        node->SetIntValue(node->GetEntryByName(value)->GetValue());
}

static void set_float(GenApi::INodeMap& nm, const char* name, double value)
{
    GenApi::CFloatPtr node = nm.GetNode(name);
    if (node && GenApi::IsWritable(node))
        node->SetValue(value);
}

static void set_bool(GenApi::INodeMap& nm, const char* name, bool value)
{
    GenApi::CBooleanPtr node = nm.GetNode(name);
    if (node && GenApi::IsWritable(node))
        node->SetValue(value);
}

static void set_int(GenApi::INodeMap& nm, const char* name, int64_t value)
{
    GenApi::CIntegerPtr node = nm.GetNode(name);
    if (node && GenApi::IsWritable(node))
        node->SetValue(value);
}

static void configure_camera(Spinnaker::CameraPtr cam, const baddy::CameraConfig& cfg)
{
    auto& nm = cam->GetNodeMap();

    set_enum(nm, "TriggerMode", "Off");

    set_enum(nm, "ExposureAuto", cfg.exposure_auto ? "Continuous" : "Off");
    if (!cfg.exposure_auto)
        set_float(nm, "ExposureTime", cfg.exposure_us);

    set_enum(nm, "GainAuto", cfg.gain_auto ? "Continuous" : "Off");
    if (!cfg.gain_auto)
        set_float(nm, "Gain", cfg.gain_db);

    set_float(nm, "BlackLevel", cfg.black_level);
    set_bool(nm, "GammaEnable", cfg.gamma_enable);

    set_enum(nm, "BalanceWhiteAuto", cfg.white_balance_auto ? "Continuous" : "Off");
    if (!cfg.white_balance_auto) {
        set_enum(nm, "BalanceRatioSelector", "Red");
        set_float(nm, "BalanceRatio", cfg.balance_ratio_red);
        set_enum(nm, "BalanceRatioSelector", "Blue");
        set_float(nm, "BalanceRatio", cfg.balance_ratio_blue);
    }

    set_enum(nm, "AdcBitDepth", cfg.adc_bit_depth.c_str());
    set_enum(nm, "SensorShutterMode", cfg.sensor_shutter_mode.c_str());

    set_bool(nm, "AcquisitionFrameRateEnable", true);
    set_float(nm, "AcquisitionFrameRate", static_cast<double>(cfg.fps));

    set_int(nm, "DeviceLinkThroughputLimit", cfg.device_link_throughput_limit);
}

struct Detection {
    bool found = false;
    double pixel_x = 0, pixel_y = 0;
};

static Detection detect_diff(const cv::Mat& prev_gray, const cv::Mat& curr_gray,
                             cv::Mat& diff_out, cv::Mat& thresh_out)
{
    Detection det;
    cv::absdiff(prev_gray, curr_gray, diff_out);
    cv::inRange(diff_out, cv::Scalar(diff_lo), cv::Scalar(diff_hi), thresh_out);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(thresh_out, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    double best_area = 0;
    int best_idx = -1;
    for (int i = 0; i < static_cast<int>(contours.size()); ++i) {
        double a = cv::contourArea(contours[i]);
        if (a >= min_area && a <= max_area && a > best_area) {
            best_area = a;
            best_idx = i;
        }
    }

    if (best_idx >= 0) {
        auto m = cv::moments(contours[best_idx]);
        if (m.m00 > 0) {
            det.found = true;
            det.pixel_x = m.m10 / m.m00;
            det.pixel_y = m.m01 / m.m00;
        }
    }
    return det;
}

int main(int argc, char** argv)
{
    std::string config_dir = "config";

    for (int i = 1; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--config") && i + 1 < argc)
            config_dir = argv[++i];
    }

    auto cfg = baddy::load_config(config_dir);

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
    configure_camera(cam, cfg.camera);

    std::string serial = cam->DeviceSerialNumber.ToString().c_str();
    std::printf("[diff_single] using camera %s\n", serial.c_str());

    cam->BeginAcquisition();
    cam_list.Clear();

    Spinnaker::ImageProcessor processor;

    cv::namedWindow("Controls", cv::WINDOW_AUTOSIZE);
    cv::createTrackbar("Diff lo",  "Controls", &diff_lo, 255);
    cv::createTrackbar("Diff hi",  "Controls", &diff_hi, 255);
    cv::createTrackbar("Min area", "Controls", &min_area, 1000);
    cv::createTrackbar("Max area", "Controls", &max_area, 10000);

    cv::namedWindow("Diff Single", cv::WINDOW_NORMAL);
    cv::resizeWindow("Diff Single", DISP_W, DISP_H * 2);

    std::puts("[diff_single] running. 'q' quit, 'h' display on/off.\n");

    cv::Mat prev_gray;
    int     frame_id  = 0;
    bool    show_disp = true;
    double  frame_us  = 0.0;

    while (true) {
        baddy::ScopedTimer frame_timer(&frame_us);

        Spinnaker::ImagePtr img;
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

        Spinnaker::ImagePtr bgr = processor.Convert(img, Spinnaker::PixelFormat_BGR8);
        cv::Mat frame(bgr->GetHeight(), bgr->GetWidth(), CV_8UC3, bgr->GetData());

        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        img->Release();

        if (prev_gray.empty()) {
            prev_gray = gray.clone();
            ++frame_id;
            continue;
        }

        // ── Detect ──
        Detection det;
        cv::Mat diff, thresh;
        double detect_us = 0;
        {
            baddy::ScopedTimer t(&detect_us);
            det = detect_diff(prev_gray, gray, diff, thresh);
        }

        prev_gray = gray.clone();

        // ── Build display ──
        double disp_us = 0;
        if (show_disp) {
            baddy::ScopedTimer t(&disp_us);

            const double sx = double(DISP_W) / frame.cols;
            const double sy = double(DISP_H) / frame.rows;

            cv::Mat disp_cam, disp_thresh;
            cv::resize(frame, disp_cam, {DISP_W, DISP_H});
            cv::resize(thresh, disp_thresh, {DISP_W, DISP_H});

            if (det.found) {
                int dx = int(det.pixel_x * sx), dy = int(det.pixel_y * sy);
                cv::circle(disp_cam, {dx, dy}, 8, {0, 255, 0}, 2);
            }

            if (frame_us > 0.0) {
                char fps_text[32];
                std::snprintf(fps_text, sizeof(fps_text), "%.1f FPS", 1e6 / frame_us);
                cv::putText(disp_cam, fps_text, {DISP_W - 130, 28},
                            cv::FONT_HERSHEY_SIMPLEX, 0.8, {0, 255, 0}, 2);
            }

            cv::Mat thresh_bgr, combined;
            cv::cvtColor(disp_thresh, thresh_bgr, cv::COLOR_GRAY2BGR);

            char info_text[64];
            std::snprintf(info_text, sizeof(info_text),
                          "diff=[%d, %d]  area=[%d, %d]", diff_lo, diff_hi, min_area, max_area);
            cv::putText(thresh_bgr, info_text, {10, 30},
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, {0, 255, 0}, 2);

            cv::vconcat(disp_cam, thresh_bgr, combined);
            cv::imshow("Diff Single", combined);
        }

        if (frame_id % 10 == 0) {
            std::printf("[diff] det=%.0fus disp=%.0fus  total=%.0fus (%.1f FPS)\n",
                        detect_us, disp_us, frame_us, 1e6 / frame_us);
        }

        int key = cv::waitKey(1) & 0xFF;
        if (key == 'q') break;
        if (key == 'h') {
            show_disp = !show_disp;
            std::printf("[diff_single] display: %s\n", show_disp ? "ON" : "OFF");
        }

        ++frame_id;
    }

    cam->EndAcquisition();
    cam->DeInit();
    cam = nullptr;
    cam_list.Clear();
    system->ReleaseInstance();
    cv::destroyAllWindows();
    return 0;
}
