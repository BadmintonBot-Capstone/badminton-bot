#pragma once

#include "core/types.hpp"
#include "core/config.hpp"
#include <opencv2/core.hpp>
#include <cstdint>
#include <memory>
#include <string>

// Forward-declare Spinnaker types to avoid leaking the SDK header everywhere.
namespace Spinnaker { class SystemPtr; class CameraPtr; class CameraList; }

namespace baddy {

// Synchronised stereo frame grab from two Blackfly S cameras using
// the primary–secondary GPIO hardware trigger.
class StereoCamera {
public:
    explicit StereoCamera(const CameraConfig& cfg);
    ~StereoCamera();

    StereoCamera(const StereoCamera&) = delete;
    StereoCamera& operator=(const StereoCamera&) = delete;

    // Begin acquisition (starts secondary first, then primary).
    void start();

    // Grab a synchronised pair.  Returns false on timeout / error.
    // timestamp_ns comes from the primary camera's hardware clock.
    bool grab(cv::Mat& left, cv::Mat& right, int64_t& timestamp_ns);

    // raw bayer pair plus each camera's frame counter and hardware timestamp, for recording
    struct RawPair {
        cv::Mat  left, right;   // CV_8UC1, copied into whatever these already point at when the size matches
        uint64_t left_id = 0, right_id = 0;
        int64_t  left_ts = 0, right_ts = 0;
    };
    bool grab_raw(RawPair& pair);

    // keep every frame in order with a deep queue instead of the default, call before start()
    void deep_buffers(int count);

    // settings as each camera actually applied them
    struct Applied {
        std::string serial, pixel_format;
        int    width = 0, height = 0;
        double exposure_us = 0, gain_db = 0, fps = 0;
    };
    Applied applied(bool primary) const;

    // End acquisition and release resources.
    void stop();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace baddy
