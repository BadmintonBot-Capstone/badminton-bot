// camera.cpp — Spinnaker-based stereo acquisition.
//
// This file is only compiled when WITH_SPINNAKER is ON.
// The actual Spinnaker SDK must be installed.

#include "core/camera.hpp"

#ifdef __has_include
# if __has_include(<Spinnaker.h>)
#  include <Spinnaker.h>
#  define HAS_SPINNAKER 1
# endif
#endif

#ifndef HAS_SPINNAKER
// Stub implementation when the SDK is not available (header-only build check).
namespace baddy {
struct StereoCamera::Impl {};
StereoCamera::StereoCamera(const CameraConfig&) : impl_(std::make_unique<Impl>()) {}
StereoCamera::~StereoCamera() = default;
void StereoCamera::start() {}
bool StereoCamera::grab(cv::Mat&, cv::Mat&, int64_t&) { return false; }
void StereoCamera::stop() {}
} // namespace baddy

#else // HAS_SPINNAKER — real implementation

#include <cstdio>

namespace baddy {

struct StereoCamera::Impl {
    Spinnaker::SystemPtr system;
    Spinnaker::CameraPtr primary;
    Spinnaker::CameraPtr secondary;
    CameraConfig cfg;
};

static void configure_common(Spinnaker::CameraPtr cam, const CameraConfig& cfg)
{
    auto& nodemap = cam->GetNodeMap();

    // Exposure.
    Spinnaker::GenApi::CEnumerationPtr exposure_auto = nodemap.GetNode("ExposureAuto");
    if (exposure_auto) exposure_auto->SetIntValue(exposure_auto->GetEntryByName("Off")->GetValue());

    Spinnaker::GenApi::CFloatPtr exposure_time = nodemap.GetNode("ExposureTime");
    if (exposure_time) exposure_time->SetValue(cfg.exposure_us);

    // Gain.
    Spinnaker::GenApi::CEnumerationPtr gain_auto = nodemap.GetNode("GainAuto");
    if (gain_auto) gain_auto->SetIntValue(gain_auto->GetEntryByName("Off")->GetValue());

    Spinnaker::GenApi::CFloatPtr gain = nodemap.GetNode("Gain");
    if (gain) gain->SetValue(cfg.gain_db);

    // White balance.
    Spinnaker::GenApi::CEnumerationPtr wb_auto = nodemap.GetNode("BalanceWhiteAuto");
    if (wb_auto) {
        const char* mode = cfg.white_balance_auto ? "Continuous" : "Off";
        wb_auto->SetIntValue(wb_auto->GetEntryByName(mode)->GetValue());
    }

    // Frame rate.
    Spinnaker::GenApi::CBooleanPtr fr_enable = nodemap.GetNode("AcquisitionFrameRateEnable");
    if (fr_enable) fr_enable->SetValue(true);

    Spinnaker::GenApi::CFloatPtr fr = nodemap.GetNode("AcquisitionFrameRate");
    if (fr) fr->SetValue(static_cast<double>(cfg.fps));
}

static void configure_primary(Spinnaker::CameraPtr cam, const CameraConfig& cfg)
{
    configure_common(cam, cfg);

    auto& nodemap = cam->GetNodeMap();

    // Enable 3.3 V output on Line2 (triggers the secondary camera).
    Spinnaker::GenApi::CEnumerationPtr line_sel = nodemap.GetNode("LineSelector");
    if (line_sel) line_sel->SetIntValue(line_sel->GetEntryByName("Line2")->GetValue());

    Spinnaker::GenApi::CBooleanPtr v33 = nodemap.GetNode("V3_3Enable");
    if (v33) v33->SetValue(true);
}

static void configure_secondary(Spinnaker::CameraPtr cam, const CameraConfig& cfg)
{
    configure_common(cam, cfg);

    auto& nodemap = cam->GetNodeMap();

    // Trigger on Line3 rising edge from the primary camera.
    Spinnaker::GenApi::CEnumerationPtr trig_mode = nodemap.GetNode("TriggerMode");
    trig_mode->SetIntValue(trig_mode->GetEntryByName("Off")->GetValue());

    Spinnaker::GenApi::CEnumerationPtr trig_src = nodemap.GetNode("TriggerSource");
    trig_src->SetIntValue(trig_src->GetEntryByName("Line3")->GetValue());

    Spinnaker::GenApi::CEnumerationPtr trig_overlap = nodemap.GetNode("TriggerOverlap");
    if (trig_overlap)
        trig_overlap->SetIntValue(trig_overlap->GetEntryByName("ReadOut")->GetValue());

    trig_mode->SetIntValue(trig_mode->GetEntryByName("On")->GetValue());
}

StereoCamera::StereoCamera(const CameraConfig& cfg)
    : impl_(std::make_unique<Impl>())
{
    impl_->cfg = cfg;
    impl_->system = Spinnaker::System::GetInstance();

    auto cam_list = impl_->system->GetCameras();
    if (cam_list.GetSize() < 2) {
        cam_list.Clear();
        throw std::runtime_error("Need at least 2 cameras, found " +
                                 std::to_string(cam_list.GetSize()));
    }

    if (!cfg.primary_serial.empty()) {
        impl_->primary   = cam_list.GetBySerial(cfg.primary_serial);
        impl_->secondary = cam_list.GetBySerial(cfg.secondary_serial);
    } else {
        impl_->primary   = cam_list.GetByIndex(0);
        impl_->secondary = cam_list.GetByIndex(1);
    }

    impl_->primary->Init();
    impl_->secondary->Init();

    configure_primary(impl_->primary, cfg);
    configure_secondary(impl_->secondary, cfg);

    cam_list.Clear();
}

StereoCamera::~StereoCamera() { stop(); }

void StereoCamera::start()
{
    // Secondary must start first so it is waiting for the trigger.
    impl_->secondary->BeginAcquisition();
    impl_->primary->BeginAcquisition();
    std::puts("[camera] acquisition started");
}

bool StereoCamera::grab(cv::Mat& left, cv::Mat& right, int64_t& timestamp_ns)
{
    try {
        auto img_primary   = impl_->primary->GetNextImage(1000);
        auto img_secondary = impl_->secondary->GetNextImage(1000);

        if (img_primary->IsIncomplete() || img_secondary->IsIncomplete()) {
            img_primary->Release();
            img_secondary->Release();
            return false;
        }

        timestamp_ns = static_cast<int64_t>(img_primary->GetTimeStamp());

        const int w = static_cast<int>(img_primary->GetWidth());
        const int h = static_cast<int>(img_primary->GetHeight());

        // Convert Bayer to BGR.
        auto converted_p = img_primary->Convert(Spinnaker::PixelFormat_BGR8);
        auto converted_s = img_secondary->Convert(Spinnaker::PixelFormat_BGR8);

        left  = cv::Mat(h, w, CV_8UC3, converted_p->GetData()).clone();
        right = cv::Mat(h, w, CV_8UC3, converted_s->GetData()).clone();

        img_primary->Release();
        img_secondary->Release();
        return true;

    } catch (const Spinnaker::Exception& e) {
        std::fprintf(stderr, "[camera] grab error: %s\n", e.what());
        return false;
    }
}

void StereoCamera::stop()
{
    if (impl_->primary && impl_->primary->IsStreaming())
        impl_->primary->EndAcquisition();
    if (impl_->secondary && impl_->secondary->IsStreaming())
        impl_->secondary->EndAcquisition();

    if (impl_->primary)   { impl_->primary->DeInit();   impl_->primary = nullptr; }
    if (impl_->secondary) { impl_->secondary->DeInit();  impl_->secondary = nullptr; }

    if (impl_->system) {
        impl_->system->ReleaseInstance();
        impl_->system = nullptr;
    }
}

} // namespace baddy

#endif // HAS_SPINNAKER
