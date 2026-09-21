#pragma once

#include <Spinnaker.h>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

class StereoRecorder {
public:
    explicit StereoRecorder(const std::string& dir) : dir_(dir) {
        std::filesystem::create_directories(dir_);
        raw_.open(dir_ + "/frames.raw", std::ios::binary);
        csv_.open(dir_ + "/timestamps.csv");
        csv_ << "index,ts_primary,ts_secondary,id_primary,id_secondary\n";
        worker_ = std::thread([this] { run(); });
    }

    ~StereoRecorder() {
        { std::lock_guard<std::mutex> lk(m_); closed_ = true; }
        cond_.notify_all();
        worker_.join();  // finishes writing everything still queued
    }

    // Call before Release(): copies the raw sensor data.
    void push(const Spinnaker::ImagePtr& p, const Spinnaker::ImagePtr& s,
              int64_t ts_p, int64_t ts_s) {
        if (!meta_written_) { write_metadata(p); meta_written_ = true; }
        Pair pr;
        pr.left = copy(p);
        pr.right = copy(s);
        pr.ts_p = ts_p;
        pr.ts_s = ts_s;
        pr.id_p = p->GetFrameID();
        pr.id_s = s->GetFrameID();
        { std::lock_guard<std::mutex> lk(m_); q_.push_back(std::move(pr)); }
        cond_.notify_one();
    }

private:
    struct Pair {
        std::vector<uint8_t> left, right;
        int64_t ts_p = 0, ts_s = 0;
        uint64_t id_p = 0, id_s = 0;
    };

    static std::vector<uint8_t> copy(const Spinnaker::ImagePtr& img) {
        const auto* d = static_cast<const uint8_t*>(img->GetData());
        return std::vector<uint8_t>(d, d + img->GetImageSize());
    }

    void write_metadata(const Spinnaker::ImagePtr& img) {
        std::ofstream m(dir_ + "/metadata.txt");
        m << "width=" << img->GetWidth() << "\n"
          << "height=" << img->GetHeight() << "\n"
          << "pixel_format=" << img->GetPixelFormatName().c_str() << "\n"
          << "pixel_format_id=" << static_cast<int>(img->GetPixelFormat()) << "\n"
          << "bytes_per_image=" << img->GetImageSize() << "\n";
    }

    void run() {
        uint64_t idx = 0;
        while (true) {
            Pair pr;
            {
                std::unique_lock<std::mutex> lk(m_);
                cond_.wait(lk, [&] { return !q_.empty() || closed_; });
                if (q_.empty()) return;
                pr = std::move(q_.front());
                q_.pop_front();
            }
            raw_.write(reinterpret_cast<const char*>(pr.left.data()), pr.left.size());
            raw_.write(reinterpret_cast<const char*>(pr.right.data()), pr.right.size());
            csv_ << idx++ << "," << pr.ts_p << "," << pr.ts_s << ","
                 << pr.id_p << "," << pr.id_s << "\n";
        }
    }

    std::string dir_;
    std::ofstream raw_, csv_;
    std::thread worker_;
    std::mutex m_;
    std::condition_variable cond_;
    std::deque<Pair> q_;
    bool closed_ = false;
    bool meta_written_ = false;
};