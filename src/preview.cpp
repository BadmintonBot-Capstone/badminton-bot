// preview.cpp — Real-time viewer for connected Blackfly S cameras.
// Free-running continuous mode, no GPIO triggers needed.
// Loads camera.yaml so the preview matches the real pipeline's image settings.

#include "core/config.hpp"
#include <Spinnaker.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <cstdio>
#include <vector>
#include <string>

namespace GenApi = Spinnaker::GenApi;

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

static void configure_preview(Spinnaker::CameraPtr cam, const baddy::CameraConfig& cfg)
{
    auto& nm = cam->GetNodeMap();

    // Disable trigger — free-running for preview.
    set_enum(nm, "TriggerMode", "Off");

    // Exposure.
    set_enum(nm, "ExposureAuto", cfg.exposure_auto ? "Continuous" : "Off");
    if (!cfg.exposure_auto)
        set_float(nm, "ExposureTime", cfg.exposure_us);

    // Gain.
    set_enum(nm, "GainAuto", cfg.gain_auto ? "Continuous" : "Off");
    if (!cfg.gain_auto)
        set_float(nm, "Gain", cfg.gain_db);

    // Black level & gamma.
    set_float(nm, "BlackLevel", cfg.black_level);
    set_bool(nm, "GammaEnable", cfg.gamma_enable);

    // White balance.
    set_enum(nm, "BalanceWhiteAuto", cfg.white_balance_auto ? "Continuous" : "Off");
    if (!cfg.white_balance_auto) {
        set_enum(nm, "BalanceRatioSelector", "Red");
        set_float(nm, "BalanceRatio", cfg.balance_ratio_red);
        set_enum(nm, "BalanceRatioSelector", "Blue");
        set_float(nm, "BalanceRatio", cfg.balance_ratio_blue);
    }

    // Sensor.
    set_enum(nm, "AdcBitDepth", cfg.adc_bit_depth.c_str());
    set_enum(nm, "SensorShutterMode", cfg.sensor_shutter_mode.c_str());

    // Frame rate.
    set_bool(nm, "AcquisitionFrameRateEnable", true);
    set_float(nm, "AcquisitionFrameRate", static_cast<double>(cfg.fps));

    // USB transport.
    set_int(nm, "DeviceLinkThroughputLimit", cfg.device_link_throughput_limit);
}

int main(int argc, char* argv[])
{
    std::string config_dir = "config";
    if (argc > 1) config_dir = argv[1];

    std::printf("Loading config from: %s/\n", config_dir.c_str());
    auto cfg = baddy::load_config(config_dir);

    auto system = Spinnaker::System::GetInstance();
    auto cam_list = system->GetCameras();
    const unsigned int n = cam_list.GetSize();

    if (n == 0) {
        std::puts("No cameras found.");
        cam_list.Clear();
        system->ReleaseInstance();
        return 1;
    }
    std::printf("Found %u camera(s)\n", n);

    std::vector<std::string> win_names;

    for (unsigned int i = 0; i < n; ++i) {
        auto cam = cam_list.GetByIndex(i);
        cam->Init();
        configure_preview(cam, cfg.camera);

        std::string serial = cam->DeviceSerialNumber.ToString().c_str();
        std::string name = "Camera " + std::to_string(i) + " (" + serial + ")";
        win_names.push_back(name);

        cv::namedWindow(name, cv::WINDOW_NORMAL);
        cv::resizeWindow(name, 640, 400);

        cam->BeginAcquisition();
        std::printf("Camera %u (serial %s) streaming\n", i, serial.c_str());
    }

    std::puts("Press 'q' to quit.");

    Spinnaker::ImageProcessor processor;
    processor.SetColorProcessing(Spinnaker::SPINNAKER_COLOR_PROCESSING_ALGORITHM_HQ_LINEAR);

    while (true) {
        for (unsigned int i = 0; i < n; ++i) {
            auto cam = cam_list.GetByIndex(i);
            Spinnaker::ImagePtr img = cam->GetNextImage(1000);
            if (!img->IsIncomplete()) {
                Spinnaker::ImagePtr bgr = processor.Convert(img, Spinnaker::PixelFormat_BGR8);
                cv::Mat frame(bgr->GetHeight(), bgr->GetWidth(), CV_8UC3,
                              bgr->GetData());
                cv::imshow(win_names[i], frame);
            }
            img->Release();
        }
        if (cv::waitKey(1) == 'q') break;
    }

    for (unsigned int i = 0; i < n; ++i) {
        auto cam = cam_list.GetByIndex(i);
        cam->EndAcquisition();
        cam->DeInit();
    }
    cam_list.Clear();
    system->ReleaseInstance();
    cv::destroyAllWindows();
}
