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
    Spinnaker::ImageProcessor processor;
    CameraConfig cfg;
};

static void set_enum(Spinnaker::GenApi::INodeMap& nm, const char* name, const char* value)
{
    Spinnaker::GenApi::CEnumerationPtr node = nm.GetNode(name);
    if (node && Spinnaker::GenApi::IsWritable(node))
        node->SetIntValue(node->GetEntryByName(value)->GetValue());
}

static void set_float(Spinnaker::GenApi::INodeMap& nm, const char* name, double value)
{
    Spinnaker::GenApi::CFloatPtr node = nm.GetNode(name);
    if (node && Spinnaker::GenApi::IsWritable(node))
        node->SetValue(value);
}

static void set_bool(Spinnaker::GenApi::INodeMap& nm, const char* name, bool value)
{
    Spinnaker::GenApi::CBooleanPtr node = nm.GetNode(name);
    if (node && Spinnaker::GenApi::IsWritable(node))
        node->SetValue(value);
}

static void set_int(Spinnaker::GenApi::INodeMap& nm, const char* name, int64_t value)
{
    Spinnaker::GenApi::CIntegerPtr node = nm.GetNode(name);
    if (node && Spinnaker::GenApi::IsWritable(node))
        node->SetValue(value);
}

static void configure_common(Spinnaker::CameraPtr cam, const CameraConfig& cfg)
{
    auto& nm = cam->GetNodeMap();

    // Exposure.
    set_enum(nm, "ExposureAuto", cfg.exposure_auto ? "Continuous" : "Off");
    if (!cfg.exposure_auto)
        set_float(nm, "ExposureTime", cfg.exposure_us);

    // Gain.
    set_enum(nm, "GainAuto", cfg.gain_auto ? "Continuous" : "Off");
    if (!cfg.gain_auto)
        set_float(nm, "Gain", cfg.gain_db);

    // Black level.
    set_float(nm, "BlackLevel", cfg.black_level);

    // Gamma.
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

    // Chunk data — must be configured before acquisition starts.
    if (cfg.chunk_mode_active) {
        set_bool(nm, "ChunkModeActive", false);  // Disable first to allow selector changes.
        if (cfg.chunk_timestamp) {
            set_enum(nm, "ChunkSelector", "Timestamp");
            set_bool(nm, "ChunkEnable", true);
        }
        set_bool(nm, "ChunkModeActive", true);
    }
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

        // Prefer chunk timestamp (hardware exposure time) over host-side GetTimeStamp().
        if (impl_->cfg.chunk_mode_active && impl_->cfg.chunk_timestamp) {
            auto chunk = img_primary->GetChunkData();
            timestamp_ns = static_cast<int64_t>(chunk.GetTimestamp());
        } else {
            timestamp_ns = static_cast<int64_t>(img_primary->GetTimeStamp());
        }

        const int w = static_cast<int>(img_primary->GetWidth());
        const int h = static_cast<int>(img_primary->GetHeight());

        // Convert Bayer to BGR.
        auto converted_p = impl_->processor.Convert(img_primary, Spinnaker::PixelFormat_BGR8);
        auto converted_s = impl_->processor.Convert(img_secondary, Spinnaker::PixelFormat_BGR8);

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
