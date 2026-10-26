// preview.cpp — Real-time viewer and raw recorder for one Blackfly S camera, or a
// hardware-triggered stereo pair.
// Loads camera.yaml so the preview matches the real pipeline's image settings.
//
// usage: preview [config_dir] [recording_dir] [seconds] [--stereo] [--no-display] [--no-record]
//
// --no-display: no window and no conversion for it, only the recording (needs seconds).
// --no-record: grab (and pair) without writing anything, to tell disk limits from USB limits.
//
// seconds: stop after this much footage, measured by the (primary) camera's own timestamps
// (10 s at 100 fps is exactly 1000 frames). Omit or pass 0 to run until 'q'.
//
// Without --stereo: the first camera found, free-running, recorded to primary.raw.
// With --stereo: the two cameras named by primary_serial and secondary_serial in camera.yaml
// (or the first two found, if those are empty). The primary free-runs and drives Line2; the
// secondary triggers on Line3, as in StereoCamera. Pairs go to primary.raw and secondary.raw.
//
// Every complete frame (or pair) is recorded as raw sensor data (see core/stereo_recorder.h).
// Only every PREVIEW_EVERY-th frame is displayed, so the grab loop keeps up at 100 fps.

#include "core/config.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <Spinnaker.h>
#include "core/stereo_recorder.h"
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
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

    // Disable trigger — free-running. The stereo secondary turns it back on afterwards.
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

// Stereo primary: drive 3.3 V on Line2 with every exposure, which triggers the secondary.
static void configure_primary_trigger(Spinnaker::CameraPtr cam)
{
    auto& nm = cam->GetNodeMap();
    set_enum(nm, "LineSelector", "Line2");
    set_bool(nm, "V3_3Enable", true);
}

// Stereo secondary: expose on each rising edge on Line3, from the primary.
// Trigger settings can only change while TriggerMode is off.
static void configure_secondary_trigger(Spinnaker::CameraPtr cam)
{
    auto& nm = cam->GetNodeMap();
    set_enum(nm, "TriggerMode", "Off");
    set_enum(nm, "TriggerSource", "Line3");
    set_enum(nm, "TriggerActivation", "RisingEdge");
    set_enum(nm, "TriggerOverlap", "ReadOut");
    set_enum(nm, "TriggerMode", "On");
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

static std::string serial_of(Spinnaker::CameraPtr cam)
{
    return cam->DeviceSerialNumber.ToString().c_str();
}

// Grabs the next image; on a timeout or other grab error returns null and counts the error.
static Spinnaker::ImagePtr next_image(Spinnaker::CameraPtr cam, int& errors)
{
    try {
        return cam->GetNextImage(1000);
    } catch (const Spinnaker::Exception& e) {
        // A timeout must not end the program: the recorder still has frames to write.
        std::fprintf(stderr, "Grab error (%s): %s\n", serial_of(cam).c_str(), e.what());
        ++errors;
        return nullptr;
    }
}

// Converts a raw image for display. Fastest conversion: this is only for the preview,
// the recording stays raw.
static cv::Mat to_bgr(Spinnaker::ImageProcessor& processor, const Spinnaker::ImagePtr& img)
{
    Spinnaker::ImagePtr bgr = processor.Convert(img, Spinnaker::PixelFormat_BGR8);
    return cv::Mat(bgr->GetHeight(), bgr->GetWidth(), CV_8UC3, bgr->GetData()).clone();
}

static int run_preview(int argc, char* argv[])
{
    // Positional arguments in order, plus the -- options anywhere.
    std::vector<std::string> pos;
    bool stereo = false, display = true, record = true;
    for (int i = 1; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--stereo")) stereo = true;
        else if (!std::strcmp(argv[i], "--no-display")) display = false;
        else if (!std::strcmp(argv[i], "--no-record")) record = false;
        else pos.push_back(argv[i]);
    }

    std::string config_dir = "config";
    // Use a new folder for each recording: an existing recording in the same
    // folder is overwritten.
    std::string recording_dir = "C:/Users/justi/479_recording_single";
    if (pos.size() > 0) config_dir = pos[0];
    if (pos.size() > 1) recording_dir = pos[1];
    double seconds = 0.0;
    if (pos.size() > 2) {
        try {
            seconds = std::stod(pos[2]);
        } catch (const std::exception&) {
            seconds = -1.0;
        }
        if (!(seconds >= 0.0)) {
            std::fprintf(stderr, "Invalid duration '%s': expected seconds >= 0.\n", pos[2].c_str());
            return 0;
        }
    }
    const int64_t duration_ns = static_cast<int64_t>(seconds * 1e9);
    if (!display && duration_ns == 0) {
        // without a window there is no 'q' to stop on
        std::fprintf(stderr, "--no-display needs a duration in seconds.\n");
        return 1;
    }

    std::printf("Loading config from: %s/\n", config_dir.c_str());
    auto cfg = baddy::load_config(config_dir);

    auto system = Spinnaker::System::GetInstance();
    auto cam_list = system->GetCameras();
    const unsigned int found = cam_list.GetSize();

    // cams[0] is the camera recorded to primary.raw; in stereo, cams[1] goes to secondary.raw.
    std::vector<Spinnaker::CameraPtr> cams;
    if (stereo) {
        if (found < 2) {
            std::printf("--stereo needs two cameras, found %u.\n", found);
            cam_list.Clear();
            system->ReleaseInstance();
            return 1;
        }
        if (!cfg.camera.primary_serial.empty() && !cfg.camera.secondary_serial.empty()) {
            cams.push_back(cam_list.GetBySerial(cfg.camera.primary_serial));
            cams.push_back(cam_list.GetBySerial(cfg.camera.secondary_serial));
            if (!cams[0].IsValid() || !cams[1].IsValid()) {
                std::printf("Cameras %s and %s from camera.yaml are not both connected.\n",
                            cfg.camera.primary_serial.c_str(), cfg.camera.secondary_serial.c_str());
                cams.clear();
                cam_list.Clear();
                system->ReleaseInstance();
                return 1;
            }
        } else {
            cams.push_back(cam_list.GetByIndex(0));
            cams.push_back(cam_list.GetByIndex(1));
        }
    } else {
        if (found == 0) {
            std::puts("No cameras found.");
            cam_list.Clear();
            system->ReleaseInstance();
            return 1;
        }
        if (found > 1)
            std::printf("Found %u cameras; using only the first one (pass --stereo for two).\n", found);
        cams.push_back(cam_list.GetByIndex(0));
    }

    for (auto& cam : cams) {
        cam->Init();
        configure_preview(cam, cfg.camera);
        configure_stream_buffers(cam);
    }
    if (stereo) {
        configure_primary_trigger(cams[0]);
        configure_secondary_trigger(cams[1]);
    }

    std::string win_name = "Camera (" + serial_of(cams[0]) + ")";
    if (stereo) win_name = "Stereo (primary " + serial_of(cams[0]) + " | secondary " + serial_of(cams[1]) + ")";
    if (display) {
        cv::namedWindow(win_name, cv::WINDOW_NORMAL);
        cv::resizeWindow(win_name, stereo ? 1280 : 640, 400);
    }

    // Stereo: the secondary must be waiting for the trigger before the primary starts.
    for (auto it = cams.rbegin(); it != cams.rend(); ++it)
        (*it)->BeginAcquisition();
    for (auto& cam : cams)
        std::printf("Camera (serial %s) streaming\n", serial_of(cam).c_str());

    // The rate the camera can actually deliver with the current settings.
    GenApi::CFloatPtr rate = cams[0]->GetNodeMap().GetNode("AcquisitionResultingFrameRate");
    if (rate && GenApi::IsReadable(rate))
        std::printf("Camera resulting frame rate: %.1f fps\n", rate->GetValue());

    std::unique_ptr<StereoRecorder> recorder;
    if (record) {
        recorder = std::make_unique<StereoRecorder>(recording_dir, stereo ? 150 : 300);
        std::printf("Recording %s to %s\n", stereo ? "stereo pairs" : "one camera", recording_dir.c_str());
    } else {
        std::printf("Grabbing %s without recording\n", stereo ? "stereo pairs" : "one camera");
    }
    if (duration_ns > 0)
        std::printf(display ? "Recording %.3g s; press 'q' in the preview window to stop early.\n"
                            : "Recording %.3g s, no display.\n", seconds);
    else
        std::puts("Press 'q' in the preview window to stop.");

    // Fastest conversion: this is only for the preview, the recording stays raw.
    Spinnaker::ImageProcessor processor;
    processor.SetColorProcessing(Spinnaker::SPINNAKER_COLOR_PROCESSING_ALGORITHM_NEAREST_NEIGHBOR);

    unsigned long long grabbed = 0, incomplete = 0, not_recorded = 0, resyncs = 0, missing = 0;
    uint64_t last_id = 0;
    bool have_last_id = false;
    int errors = 0;
    int64_t first_ts = -1;
    // Stereo: each camera's frame counter at the first pair, to keep the pairs in step.
    uint64_t first_id_p = 0, first_id_s = 0;
    bool have_first_ids = false;
    const auto t_start = std::chrono::steady_clock::now();

    while (errors <= MAX_GRAB_ERRORS) {
        Spinnaker::ImagePtr img = next_image(cams[0], errors);
        if (!img.IsValid()) continue;
        Spinnaker::ImagePtr img_s;
        if (stereo) {
            img_s = next_image(cams[1], errors);
            if (!img_s.IsValid()) { img->Release(); continue; }

            // A frame lost by one camera would shift every later pair by one, so whichever
            // camera is behind skips ahead until both counters have moved by the same amount.
            if (have_first_ids) {
                auto step = [&](const Spinnaker::ImagePtr& a, uint64_t first) {
                    return static_cast<int64_t>(a->GetFrameID() - first);
                };
                bool in_step = false;
                for (int tries = 0; tries < 3; ++tries) {
                    const int64_t dp = step(img, first_id_p), ds = step(img_s, first_id_s);
                    if (dp == ds) { in_step = true; break; }
                    ++resyncs;
                    Spinnaker::ImagePtr& behind = dp < ds ? img : img_s;
                    behind->Release();
                    behind = next_image(dp < ds ? cams[0] : cams[1], errors);
                    if (!behind.IsValid()) break;
                }
                if (!in_step && img.IsValid() && img_s.IsValid())
                    in_step = step(img, first_id_p) == step(img_s, first_id_s);
                if (!in_step) {
                    // Still apart: drop this pair rather than record two different moments.
                    if (img.IsValid()) img->Release();
                    if (img_s.IsValid()) img_s->Release();
                    continue;
                }
            }
        }

        const bool complete = !img->IsIncomplete() && (!stereo || !img_s->IsIncomplete());
        if (complete && duration_ns > 0) {
            // Stop on the first frame past the duration, without recording it.
            const auto ts = static_cast<int64_t>(img->GetTimeStamp());
            if (first_ts < 0) first_ts = ts;
            if (ts - first_ts >= duration_ns) {
                img->Release();
                if (img_s.IsValid()) img_s->Release();
                break;
            }
        }
        if (complete) {
            if (stereo && !have_first_ids) {
                first_id_p = img->GetFrameID();
                first_id_s = img_s->GetFrameID();
                have_first_ids = true;
            }
            // Frames the camera made but this loop never got, from gaps in its frame counter.
            const uint64_t id = img->GetFrameID();
            if (have_last_id && id > last_id + 1) missing += id - last_id - 1;
            last_id = id;
            have_last_id = true;

            if (recorder) {
                const bool ok = stereo
                    ? recorder->push(img, img_s, static_cast<int64_t>(img->GetTimeStamp()),
                                     static_cast<int64_t>(img_s->GetTimeStamp()))
                    : recorder->push(img, static_cast<int64_t>(img->GetTimeStamp()));
                if (!ok) ++not_recorded;
            }
        } else {
            ++incomplete;
        }

        const bool show = display && (grabbed % PREVIEW_EVERY == 0);
        if (show && complete) {
            cv::Mat frame = to_bgr(processor, img);
            if (stereo) {
                // Side by side at half size each, so the window costs no more than one camera.
                cv::Mat left, right;
                cv::resize(frame, left, {}, 0.5, 0.5, cv::INTER_NEAREST);
                cv::resize(to_bgr(processor, img_s), right, {}, 0.5, 0.5, cv::INTER_NEAREST);
                cv::hconcat(left, right, frame);
            }
            cv::imshow(win_name, frame);
        }

        img->Release();
        if (img_s.IsValid()) img_s->Release();
        ++grabbed;

        // waitKey itself costs a few milliseconds, so it only runs on preview frames.
        if (show && cv::waitKey(1) == 'q') break;
    }
    if (errors > MAX_GRAB_ERRORS)
        std::fprintf(stderr, "Stopping after %d grab errors.\n", errors);

    const double elapsed_s =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();

    for (auto& cam : cams)
        cam->EndAcquisition();
    for (auto& cam : cams)
        cam->DeInit();
    cams.clear();        // every camera reference must be gone before ReleaseInstance()
    recorder.reset();    // writes any frames still queued, then closes the files
    cam_list.Clear();
    system->ReleaseInstance();
    cv::destroyAllWindows();

    std::printf("Grabbed %llu %s in %.1f s (%.1f fps): %llu incomplete, %llu not recorded\n",
                grabbed, stereo ? "pairs" : "frames", elapsed_s,
                elapsed_s > 0 ? grabbed / elapsed_s : 0.0, incomplete, not_recorded);
    std::printf("%llu %s missing (gaps in the %scamera's frame counter)\n", missing,
                stereo ? "pairs" : "frames", stereo ? "primary " : "");
    if (stereo && resyncs)
        std::printf("%llu frames skipped to keep the pairs in step: check the trigger cable "
                    "and the USB bandwidth.\n", resyncs);
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
