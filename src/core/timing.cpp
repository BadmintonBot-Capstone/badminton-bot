#include "core/timing.hpp"
#include <algorithm>
#include <cstdio>
#include <vector>

namespace baddy {

TimingLog::TimingLog(std::size_t window_size) : window_size_(window_size) {}

void TimingLog::record(const FrameTiming& ft)
{
    samples_.push_back(ft);
    if (samples_.size() > window_size_)
        samples_.pop_front();
}

static double percentile(std::vector<double>& v, double p)
{
    if (v.empty()) return 0.0;
    std::sort(v.begin(), v.end());
    const std::size_t idx = std::min<std::size_t>(
        static_cast<std::size_t>(p / 100.0 * static_cast<double>(v.size())),
        v.size() - 1);
    return v[idx];
}

std::string TimingLog::summary() const
{
    if (samples_.empty()) return "(no samples)";

    const std::size_t n = samples_.size();
    std::vector<double> totals(n);
    double mean_total = 0.0;

    for (std::size_t i = 0; i < n; ++i) {
        totals[i] = samples_[i].total_us;
        mean_total += totals[i];
    }
    mean_total /= static_cast<double>(n);
    const double p95 = percentile(totals, 95.0);

    char buf[256];
    std::snprintf(buf, sizeof(buf),
                  "frames=%zu  mean=%.0f µs  p95=%.0f µs",
                  n, mean_total, p95);
    return buf;
}

} // namespace baddy
