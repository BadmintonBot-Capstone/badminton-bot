#pragma once

#include "core/types.hpp"
#include <chrono>
#include <deque>
#include <string>

namespace baddy {

// RAII scoped timer.  Writes elapsed microseconds to *out on destruction.
class ScopedTimer {
public:
    explicit ScopedTimer(double* out)
        : out_(out), start_(std::chrono::steady_clock::now()) {}
    ~ScopedTimer() {
        auto end = std::chrono::steady_clock::now();
        *out_ = std::chrono::duration<double, std::micro>(end - start_).count();
    }
    ScopedTimer(const ScopedTimer&) = delete;
    ScopedTimer& operator=(const ScopedTimer&) = delete;

private:
    double* out_;
    std::chrono::steady_clock::time_point start_;
};

// Rolling window of FrameTiming samples with summary statistics.
class TimingLog {
public:
    explicit TimingLog(std::size_t window_size = 500);

    void record(const FrameTiming& ft);

    // Print a one-line summary of mean and p95 for each stage.
    std::string summary() const;

private:
    std::size_t window_size_;
    std::deque<FrameTiming> samples_;
};

} // namespace baddy
