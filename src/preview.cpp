// preview.cpp — Real-time viewer and raw recorder for ONE Blackfly S camera.
// Free-running continuous mode, no GPIO triggers needed.
// Loads camera.yaml so the preview matches the real pipeline's image settings.
//
// usage: preview [config_dir] [recording_dir] [seconds]
//
// seconds: stop after this much footage, measured by the camera's own timestamps
// (10 s at 100 fps is exactly 1000 frames). Omit or pass 0 to run until 'q'.
//
// Every complete frame is recorded as raw sensor data (see core/stereo_recorder.h).
// Only every PREVIEW_EVERY-th frame is displayed, so the grab loop keeps up at 100 fps.

#include "core/config.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <Spinnaker.h>
#include "core/stereo_recorder.h"
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <memory>
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

    // USB transport. Set before the frame rate: a low limit caps the maximum frame rate.
    set_int(nm, "DeviceLinkThroughputLimit", cfg.device_link_throughput_limit);

    // Frame rate.
    set_bool(nm, "AcquisitionFrameRateEnable", true);
    set_float(nm, "AcquisitionFrameRate", static_cast<double>(cfg.fps));
}

// Show only every Nth frame. Converting and drawing every frame takes longer than
// the 10 ms available per frame at 100 fps, which makes the camera drop frames.
static const unsigned int PREVIEW_EVERY = 5;

// Number of images the driver may hold while the loop is briefly busy.
static const int64_t STREAM_BUFFERS = 100;

// Give up after this many failed grabs (each one waits up to a second).
static const int MAX_GRAB_ERRORS = 20;

// Give the driver more room to queue images, so a short stall in the loop
// does not lose frames. Must be called before BeginAcquisition().
static void configure_stream_buffers(Spinnaker::CameraPtr cam)
{
    auto& snm = cam->GetTLStreamNodeMap();
    set_enum(snm, "StreamBufferHandlingMode", "OldestFirst");
    set_enum(snm, "StreamBufferCountMode", "Manual");
    GenApi::CIntegerPtr count = snm.GetNode("StreamBufferCountManual");
    if (count && GenApi::IsWritable(count))
        count->SetValue(std::min<int64_t>(STREAM_BUFFERS, count->GetMax()));
}

static int run_preview(int argc, char* argv[])
{
    std::string config_dir = "config";
    // Use a new folder for each recording: an existing recording in the same
    // folder is overwritten.
    std::string recording_dir = "C:/Users/justi/479_recording_single";
    if (argc > 1) config_dir = argv[1];
    if (argc > 2) recording_dir = argv[2];
    double seconds = 0.0;
    if (argc > 3) {
        try {
            seconds = std::stod(argv[3]);
        } catch (const std::exception&) {
            seconds = -1.0;
        }
        if (!(seconds >= 0.0)) {
            std::fprintf(stderr, "Invalid duration '%s': expected seconds >= 0.\n", argv[3]);
            return 0;
        }
    }
    const int64_t duration_ns = static_cast<int64_t>(seconds * 1e9);

    std::printf("Loading config from: %s/\n", config_dir.c_str());
    auto cfg = baddy::load_config(config_dir);

    auto system = Spinnaker::System::GetInstance();
    auto cam_list = system->GetCameras();
    const unsigned int found = cam_list.GetSize();

    if (found == 0) {
        std::puts("No cameras found.");
        cam_list.Clear();
        system->ReleaseInstance();
        return 1;
    }
    if (found > 1)
        std::printf("Found %u cameras; using only the first one.\n", found);

    Spinnaker::CameraPtr cam = cam_list.GetByIndex(0);
    cam->Init();
    configure_preview(cam, cfg.camera);
    configure_stream_buffers(cam);

    const std::string serial = cam->DeviceSerialNumber.ToString().c_str();
    const std::string win_name = "Camera (" + serial + ")";
    cv::namedWindow(win_name, cv::WINDOW_NORMAL);
    cv::resizeWindow(win_name, 640, 400);

    cam->BeginAcquisition();
    std::printf("Camera (serial %s) streaming\n", serial.c_str());

    // The rate the camera can actually deliver with the current settings.
    GenApi::CFloatPtr rate = cam->GetNodeMap().GetNode("AcquisitionResultingFrameRate");
    if (rate && GenApi::IsReadable(rate))
        std::printf("Camera resulting frame rate: %.1f fps\n", rate->GetValue());

    auto recorder = std::make_unique<StereoRecorder>(recording_dir);
    std::printf("Recording to %s\n", recording_dir.c_str());
    if (duration_ns > 0)
        std::printf("Recording %.3g s; press 'q' in the preview window to stop early.\n", seconds);
    else
        std::puts("Press 'q' in the preview window to stop.");

    // Fastest conversion: this is only for the preview, the recording stays raw.
    Spinnaker::ImageProcessor processor;
    processor.SetColorProcessing(Spinnaker::SPINNAKER_COLOR_PROCESSING_ALGORITHM_NEAREST_NEIGHBOR);

    unsigned long long grabbed = 0, incomplete = 0, not_recorded = 0;
    int errors = 0;
    int64_t first_ts = -1;
    const auto t_start = std::chrono::steady_clock::now();

    while (true) {
        Spinnaker::ImagePtr img;
        try {
            img = cam->GetNextImage(1000);
        } catch (const Spinnaker::Exception& e) {
            // A timeout must not end the program: the recorder still has frames to write.
            std::fprintf(stderr, "Grab error: %s\n", e.what());
            if (++errors > MAX_GRAB_ERRORS) break;
            continue;
        }

        const bool complete = !img->IsIncomplete();
        if (complete && duration_ns > 0) {
            // Stop on the first frame past the duration, without recording it.
            const auto ts = static_cast<int64_t>(img->GetTimeStamp());
            if (first_ts < 0) first_ts = ts;
            if (ts - first_ts >= duration_ns) {
                img->Release();
                break;
            }
        }
        if (complete) {
            if (!recorder->push(img, static_cast<int64_t>(img->GetTimeStamp())))
                ++not_recorded;
        } else {
            ++incomplete;
        }

        const bool show = (grabbed % PREVIEW_EVERY == 0);
        if (show && complete) {
            Spinnaker::ImagePtr bgr = processor.Convert(img, Spinnaker::PixelFormat_BGR8);
            cv::Mat frame(bgr->GetHeight(), bgr->GetWidth(), CV_8UC3, bgr->GetData());
            cv::imshow(win_name, frame);
        }

        img->Release();
        ++grabbed;

        // waitKey itself costs a few milliseconds, so it only runs on preview frames.
        if (show && cv::waitKey(1) == 'q') break;
    }

    const double elapsed_s =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();

    cam->EndAcquisition();
    cam->DeInit();
    cam = nullptr;       // every camera reference must be gone before ReleaseInstance()
    recorder.reset();    // writes any frames still queued, then closes the files
    cam_list.Clear();
    system->ReleaseInstance();
    cv::destroyAllWindows();

    std::printf("Grabbed %llu frames in %.1f s (%.1f fps): %llu incomplete, %llu not recorded\n",
                grabbed, elapsed_s, elapsed_s > 0 ? grabbed / elapsed_s : 0.0,
                incomplete, not_recorded);
    return 0;
}

int main(int argc, char* argv[])
{
    // Unbuffered, so messages are not lost if the program dies.
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    try {
        return run_preview(argc, argv);
    } catch (const Spinnaker::Exception& e) {
        std::fprintf(stderr, "Spinnaker error: %s\n", e.what());
    } catch (const std::exception& e) {
        std::fprintf(stderr, "Error: %s\n", e.what());
    }
    return 1;
}
