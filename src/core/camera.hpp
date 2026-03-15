#pragma once

#include "core/types.hpp"
#include "core/config.hpp"
#include <opencv2/core.hpp>
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

    // End acquisition and release resources.
    void stop();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace baddy
