// free-running single camera setup, resets to the default user set so spinview leftovers can't leak in

#pragma once

#include "core/config.hpp"
#include <Spinnaker.h>
#include <algorithm>
#include <cstdio>
#include <string>

namespace single_cam {

namespace GenApi = Spinnaker::GenApi;

inline void set_enum(GenApi::INodeMap& nm, const char* name, const char* value)
{
    GenApi::CEnumerationPtr node = nm.GetNode(name);
    if (node && GenApi::IsWritable(node))
        node->SetIntValue(node->GetEntryByName(value)->GetValue());
}

inline void set_float(GenApi::INodeMap& nm, const char* name, double value)
{
    GenApi::CFloatPtr node = nm.GetNode(name);
    if (node && GenApi::IsWritable(node))
        node->SetValue(value);
}

inline void set_bool(GenApi::INodeMap& nm, const char* name, bool value)
{
    GenApi::CBooleanPtr node = nm.GetNode(name);
    if (node && GenApi::IsWritable(node))
        node->SetValue(value);
}

inline void set_int(GenApi::INodeMap& nm, const char* name, int64_t value)
{
    GenApi::CIntegerPtr node = nm.GetNode(name);
    if (node && GenApi::IsWritable(node))
        node->SetValue(value);
}

inline double get_float(GenApi::INodeMap& nm, const char* name)
{
    GenApi::CFloatPtr node = nm.GetNode(name);
    return (node && GenApi::IsReadable(node)) ? node->GetValue() : -1.0;
}

// settings as the camera actually applied them
struct Applied {
    std::string serial;
    int         width = 0, height = 0;
    double      exposure_us = 0, gain_db = 0, fps = 0;
};

inline Applied configure(Spinnaker::CameraPtr cam, const baddy::CameraConfig& cfg)
{
    auto& nm = cam->GetNodeMap();

    set_enum(nm, "UserSetSelector", "Default");
    GenApi::CCommandPtr load = nm.GetNode("UserSetLoad");
    if (load && GenApi::IsWritable(load)) load->Execute();

    set_enum(nm, "TriggerMode", "Off");
    set_enum(nm, "PixelFormat", cfg.pixel_format.c_str());
    set_enum(nm, "AdcBitDepth", cfg.adc_bit_depth.c_str());
    set_enum(nm, "SensorShutterMode", cfg.sensor_shutter_mode.c_str());

    set_enum(nm, "ExposureAuto", cfg.exposure_auto ? "Continuous" : "Off");
    if (!cfg.exposure_auto) set_float(nm, "ExposureTime", cfg.exposure_us);

    set_enum(nm, "GainAuto", cfg.gain_auto ? "Continuous" : "Off");
    if (!cfg.gain_auto) set_float(nm, "Gain", cfg.gain_db);

    set_float(nm, "BlackLevel", cfg.black_level);
    set_bool(nm, "GammaEnable", cfg.gamma_enable);

    set_enum(nm, "BalanceWhiteAuto", cfg.white_balance_auto ? "Continuous" : "Off");
    if (!cfg.white_balance_auto) {
        set_enum(nm, "BalanceRatioSelector", "Red");
        set_float(nm, "BalanceRatio", cfg.balance_ratio_red);
        set_enum(nm, "BalanceRatioSelector", "Blue");
        set_float(nm, "BalanceRatio", cfg.balance_ratio_blue);
    }

    set_int(nm, "DeviceLinkThroughputLimit", cfg.device_link_throughput_limit);
    set_bool(nm, "AcquisitionFrameRateEnable", true);
    set_float(nm, "AcquisitionFrameRate", static_cast<double>(cfg.fps));

    // oldest first so no frame is skipped while the host is busy.
    auto& snm = cam->GetTLStreamNodeMap();
    set_enum(snm, "StreamBufferHandlingMode", "OldestFirst");
    set_enum(snm, "StreamBufferCountMode", "Manual");
    GenApi::CIntegerPtr count = snm.GetNode("StreamBufferCountManual");
    if (count && GenApi::IsWritable(count)) {
        count->SetValue(std::min<int64_t>(64, count->GetMax()));
        if (count->GetValue() < 16)
            std::printf("[cam] only %lld stream buffers, raise usbfs_memory_mb to 1000\n",
                        static_cast<long long>(count->GetValue()));
    }

    Applied a;
    a.serial      = cam->DeviceSerialNumber.ToString().c_str();
    a.width       = static_cast<int>(cam->Width.GetValue());
    a.height      = static_cast<int>(cam->Height.GetValue());
    a.exposure_us = get_float(nm, "ExposureTime");
    a.gain_db     = get_float(nm, "Gain");
    a.fps         = get_float(nm, "AcquisitionResultingFrameRate");
    std::printf("[cam %s] %dx%d %s  exposure=%.0f us  gain=%.1f dB  fps=%.1f\n",
                a.serial.c_str(), a.width, a.height, cfg.pixel_format.c_str(),
                a.exposure_us, a.gain_db, a.fps);
    return a;
}

} // namespace single_cam
