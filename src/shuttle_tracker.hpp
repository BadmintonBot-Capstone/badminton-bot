// motion based shuttle detection for one camera, a running background plus a multi track manager

#pragma once

#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <string>
#include <vector>

namespace shuttle {

struct Params {
    // detection, all in half resolution pixels
    double alpha     = 0.05;   // background update rate per frame
    double blur      = 1.5;    // gaussian sigma applied to |frame - background|
    double thresh    = 12;     // grey levels
    int    min_area  = 4;
    int    max_area  = 900;    // anything bigger is treated as a person
    int    near_big  = 40;     // no new tracks this close to a person

    // tracking
    double gate          = 25;
    double gate_v        = 0.6;
    double spawn_r       = 150;
    int    confirm_hits  = 4;
    double confirm_speed = 8;    // px per frame
    double still_speed   = 3;
    int    still_frames  = 8;
    int    coast         = 4;
    int    window        = 8;
    double max_jerk      = 0.5;
    double min_travel    = 60;
};

inline Params load_params(const std::string& path)
{
    Params p;
    if (!std::filesystem::exists(path)) return p;
    YAML::Node n = YAML::LoadFile(path);
    auto rd = [&](const char* k, auto& v) { if (n[k]) v = n[k].as<std::decay_t<decltype(v)>>(); };
    rd("alpha", p.alpha); rd("blur", p.blur); rd("thresh", p.thresh);
    rd("min_area", p.min_area); rd("max_area", p.max_area); rd("near_big", p.near_big);
    rd("gate", p.gate); rd("gate_v", p.gate_v); rd("spawn_r", p.spawn_r);
    rd("confirm_hits", p.confirm_hits); rd("confirm_speed", p.confirm_speed);
    rd("still_speed", p.still_speed); rd("still_frames", p.still_frames); rd("coast", p.coast);
    rd("window", p.window); rd("max_jerk", p.max_jerk); rd("min_travel", p.min_travel);
    return p;
}

struct Candidate {
    cv::Point2d c;
    int  area = 0;
    bool near_big = false;
};

// bayer frame in, blob candidates out
class Detector {
public:
    explicit Detector(const Params& p) : p_(p) {}

    const std::vector<Candidate>& step(const cv::Mat& bayer)
    {
        // 2x2 area average of the raw bayer mosaic gives a half resolution grey image
        cv::resize(bayer, half_, {bayer.cols / 2, bayer.rows / 2}, 0, 0, cv::INTER_AREA);
        half_.convertTo(g_, CV_32F);
        if (bg_.empty()) g_.copyTo(bg_);

        cv::absdiff(g_, bg_, diff_);
        cv::GaussianBlur(diff_, diff_, {0, 0}, p_.blur);
        cv::threshold(diff_, mask_, p_.thresh, 255, cv::THRESH_BINARY);
        mask_.convertTo(mask8_, CV_8U);
        cv::accumulateWeighted(g_, bg_, p_.alpha);

        int n = cv::connectedComponentsWithStats(mask8_, labels_, stats_, cents_, 8, CV_32S);
        big_.clear();
        for (int k = 1; k < n; ++k) {
            if (stats_.at<int>(k, cv::CC_STAT_AREA) > p_.max_area) {
                cv::Rect r(stats_.at<int>(k, cv::CC_STAT_LEFT), stats_.at<int>(k, cv::CC_STAT_TOP),
                           stats_.at<int>(k, cv::CC_STAT_WIDTH), stats_.at<int>(k, cv::CC_STAT_HEIGHT));
                big_.push_back(r + cv::Size(2 * p_.near_big, 2 * p_.near_big) - cv::Point(p_.near_big, p_.near_big));
            }
        }
        cands_.clear();
        for (int k = 1; k < n; ++k) {
            int a = stats_.at<int>(k, cv::CC_STAT_AREA);
            if (a < p_.min_area || a > p_.max_area) continue;
            Candidate c{{cents_.at<double>(k, 0), cents_.at<double>(k, 1)}, a, false};
            for (auto& r : big_)
                if (r.contains(cv::Point(int(c.c.x), int(c.c.y)))) c.near_big = true;
            cands_.push_back(c);
        }
        return cands_;
    }

    const cv::Mat& grey() const { return half_; }
    const cv::Mat& mask() const { return mask8_; }
    const std::vector<cv::Rect>& people() const { return big_; }

private:
    Params p_;
    cv::Mat half_, g_, bg_, diff_, mask_, mask8_, labels_, stats_, cents_;
    std::vector<cv::Rect> big_;
    std::vector<Candidate> cands_;
};

struct Track {
    cv::Point2d p, v, origin;
    bool   has_v = false, confirmed = false;
    int    hits = 1, miss = 0, still = 0;
    std::vector<double>      speeds;
    std::vector<cv::Point2d> steps;
    std::vector<cv::Point2d> trail;
};

enum class State { Search, Hit, Coast };

struct Output {
    State       state = State::Search;
    cv::Point2d p;
    const Track* track = nullptr;
};

// keeps every plausible track alive and reports the best fast, smooth, travelling one
class Tracker {
public:
    Tracker(const Params& p, int w, int h) : p_(p), w_(w), h_(h) {}

    Output step(const std::vector<Candidate>& cands)
    {
        struct Pair { double d; int t, c; };
        std::vector<Pair> pairs;
        for (int ti = 0; ti < int(tracks_.size()); ++ti) {
            auto& t = tracks_[ti];
            cv::Point2d pr = t.has_v ? t.p + t.v : t.p;
            double r = t.has_v ? p_.gate + p_.gate_v * cv::norm(t.v) : p_.spawn_r;
            for (int ci = 0; ci < int(cands.size()); ++ci) {
                double d = cv::norm(cands[ci].c - pr);
                if (d < r) pairs.push_back({d, ti, ci});
            }
        }
        std::sort(pairs.begin(), pairs.end(), [&](const Pair& a, const Pair& b) {
            bool ca = tracks_[a.t].confirmed, cb = tracks_[b.t].confirmed;
            return ca != cb ? ca : a.d < b.d;
        });

        std::vector<char> used_t(tracks_.size(), 0), used_c(cands.size(), 0);
        for (auto& pr : pairs) {
            if (used_t[pr.t] || used_c[pr.c]) continue;
            used_t[pr.t] = used_c[pr.c] = 1;
            auto& t = tracks_[pr.t];
            cv::Point2d step = cands[pr.c].c - t.p;
            t.v = t.has_v ? 0.5 * t.v + 0.5 * step : step;
            t.has_v = true;
            t.p = cands[pr.c].c;
            ++t.hits;
            t.miss = 0;
            t.speeds.push_back(cv::norm(step));
            t.steps.push_back(step);
            t.trail.push_back(t.p);
            if (!t.confirmed && t.hits >= p_.confirm_hits && median(t.speeds) >= p_.confirm_speed)
                t.confirmed = true;
            t.still = cv::norm(t.v) < p_.still_speed ? t.still + 1 : 0;
        }

        std::vector<Track> keep;
        for (int ti = 0; ti < int(tracks_.size()); ++ti) {
            auto& t = tracks_[ti];
            if (!used_t[ti]) {
                ++t.miss;
                if (t.has_v) t.p += t.v;
            }
            bool off = t.p.x < -20 || t.p.x > w_ + 20 || t.p.y < -20 || t.p.y > h_ + 20;
            int  limit = t.confirmed ? p_.coast : 1;
            if (t.miss > limit || off || t.still > p_.still_frames) continue;
            if (!t.confirmed && t.hits >= 8) continue;
            keep.push_back(std::move(t));
        }
        tracks_ = std::move(keep);

        for (int ci = 0; ci < int(cands.size()); ++ci) {
            if (used_c[ci] || cands[ci].near_big) continue;
            Track t;
            t.p = t.origin = cands[ci].c;
            t.trail.push_back(t.p);
            tracks_.push_back(std::move(t));
        }

        Output out;
        for (auto& t : tracks_) {
            if (!good(t)) continue;
            if (!out.track || t.hits > out.track->hits) {
                out.track = &t;
                out.p     = t.p;
                out.state = t.miss == 0 ? State::Hit : State::Coast;
            }
        }
        return out;
    }

    size_t live_tracks() const { return tracks_.size(); }

private:
    bool good(const Track& t) const
    {
        if (t.hits < p_.confirm_hits || t.speeds.empty()) return false;
        int n = std::min<int>(p_.window, int(t.speeds.size()));
        std::vector<double> sp(t.speeds.end() - n, t.speeds.end());
        double med_speed = median(sp);
        if (med_speed < p_.confirm_speed) return false;
        if (cv::norm(t.p - t.origin) < p_.min_travel) return false;
        if (n < 2) return true;
        std::vector<double> jerk;
        for (size_t i = t.steps.size() - n + 1; i < t.steps.size(); ++i)
            jerk.push_back(cv::norm(t.steps[i] - t.steps[i - 1]));
        return median(jerk) <= p_.max_jerk * med_speed;
    }

    static double median(std::vector<double> v)
    {
        std::sort(v.begin(), v.end());
        size_t n = v.size();
        return n % 2 ? v[n / 2] : 0.5 * (v[n / 2 - 1] + v[n / 2]);
    }

    Params p_;
    int w_, h_;
    std::vector<Track> tracks_;
};

} // namespace shuttle
