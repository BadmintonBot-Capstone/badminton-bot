#pragma once

#include <Spinnaker.h>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// Records one camera or a stereo pair into a folder, one raw file per camera:
//
//   primary.raw     raw sensor data of the primary camera, images back to back
//   secondary.raw   the same for the secondary camera (stereo recordings only)
//   timestamps.csv  one row per push; row N describes image N of each raw file
//   metadata.txt    image size and pixel format, written on the first push
//
// A recording is either single or stereo. The first push decides which, and
// pushes of the other kind are rejected, so the raw files always stay aligned
// with the CSV.
//
// push() copies the image and returns; a background thread does the disk writes.
// Image buffers are recycled, so after the first few frames push() makes no
// memory allocations. Call push() from one thread only.
class StereoRecorder {
public:
    // max_queued: records allowed to wait for the disk before new ones are dropped.
    // At 1920x1200 8-bit, 300 records is about 0.7 GB (single) or 1.4 GB (stereo).
    explicit StereoRecorder(const std::string& dir, size_t max_queued = 300)
        : dir_(dir), max_queued_(max_queued) {
        std::filesystem::create_directories(dir_);
        raw_p_.open(dir_ + "/primary.raw", std::ios::binary);
        csv_.open(dir_ + "/timestamps.csv");
        csv_ << "index,ts_primary,ts_secondary,id_primary,id_secondary,offset_drift_ns\n";
        worker_ = std::thread([this] { run(); });
    }

    ~StereoRecorder() {
        { std::lock_guard<std::mutex> lk(m_); closed_ = true; }
        cond_.notify_all();
        worker_.join();  // finishes writing everything still queued
        if (dropped_ || rejected_)
            std::fprintf(stderr,
                         "[recorder] %llu records dropped (disk too slow), "
                         "%llu rejected (wrong kind of push or unexpected image size)\n",
                         static_cast<unsigned long long>(dropped_),
                         static_cast<unsigned long long>(rejected_));
    }

    StereoRecorder(const StereoRecorder&) = delete;
    StereoRecorder& operator=(const StereoRecorder&) = delete;

    // Stereo: one image from each camera. Call before Release().
    // Returns false if the pair was not recorded.
    bool push(const Spinnaker::ImagePtr& p, const Spinnaker::ImagePtr& s,
              int64_t ts_p, int64_t ts_s) {
        return enqueue(p, &s, ts_p, ts_s);
    }

    // Single camera: one image. Call before Release().
    // Returns false if the image was not recorded.
    bool push(const Spinnaker::ImagePtr& p, int64_t ts_p) {
        return enqueue(p, nullptr, ts_p, 0);
    }

private:
    struct Record {
        std::vector<uint8_t> left, right;  // right stays empty in a single recording
        int64_t ts_p = 0, ts_s = 0;
        uint64_t id_p = 0, id_s = 0;
        bool stereo = false;
    };

    // Copies the image into buf, reusing buf's memory when it is already big enough.
    static void fill(std::vector<uint8_t>& buf, const Spinnaker::ImagePtr& img) {
        const auto* d = static_cast<const uint8_t*>(img->GetData());
        buf.assign(d, d + img->GetImageSize());
    }

    // Returns a recycled buffer if one is available. Call with m_ held.
    std::vector<uint8_t> take_buffer() {
        if (pool_.empty()) return {};
        std::vector<uint8_t> b = std::move(pool_.back());
        pool_.pop_back();
        return b;
    }

    bool enqueue(const Spinnaker::ImagePtr& p, const Spinnaker::ImagePtr* s,
                 int64_t ts_p, int64_t ts_s) {
        const bool stereo = (s != nullptr);

        if (!started_) {  // the first push fixes the kind of recording and the image sizes
            stereo_ = stereo;
            bytes_p_ = p->GetImageSize();
            bytes_s_ = stereo ? (*s)->GetImageSize() : 0;
            write_metadata(p, s);
            started_ = true;
        }

        // Anything that would break the fixed-size layout is refused as a whole,
        // so both raw files and the CSV always have the same number of entries.
        if (stereo != stereo_ || p->GetImageSize() != bytes_p_ ||
            (stereo && (*s)->GetImageSize() != bytes_s_)) {
            ++rejected_;
            return false;
        }

        Record r;
        r.stereo = stereo;
        {
            std::lock_guard<std::mutex> lk(m_);
            if (q_.size() >= max_queued_) {
                ++dropped_;
                return false;
            }
            r.left = take_buffer();
            if (stereo) r.right = take_buffer();
        }

        fill(r.left, p);
        r.ts_p = ts_p;
        r.id_p = p->GetFrameID();
        if (stereo) {
            fill(r.right, *s);
            r.ts_s = ts_s;
            r.id_s = (*s)->GetFrameID();
        }
        { std::lock_guard<std::mutex> lk(m_); q_.push_back(std::move(r)); }
        cond_.notify_one();
        return true;
    }

    static void write_image_info(std::ofstream& m, const std::string& prefix,
                                 const Spinnaker::ImagePtr& img) {
        m << prefix << "width=" << img->GetWidth() << "\n"
          << prefix << "height=" << img->GetHeight() << "\n"
          << prefix << "pixel_format=" << img->GetPixelFormatName().c_str() << "\n"
          << prefix << "pixel_format_id=" << static_cast<int>(img->GetPixelFormat()) << "\n"
          << prefix << "bytes_per_image=" << img->GetImageSize() << "\n";
    }

    void write_metadata(const Spinnaker::ImagePtr& p, const Spinnaker::ImagePtr* s) {
        std::ofstream m(dir_ + "/metadata.txt");
        m << "cameras=" << (s ? 2 : 1) << "\n";
        write_image_info(m, "", p);               // primary: same keys as before
        if (s) write_image_info(m, "secondary_", *s);
    }

    void run() {
        uint64_t idx = 0;
        int64_t first_offset = 0;
        while (true) {
            Record r;
            {
                std::unique_lock<std::mutex> lk(m_);
                cond_.wait(lk, [&] { return !q_.empty() || closed_; });
                if (q_.empty()) return;  // closed and fully written
                r = std::move(q_.front());
                q_.pop_front();
            }

            raw_p_.write(reinterpret_cast<const char*>(r.left.data()),
                         static_cast<std::streamsize>(r.left.size()));
            csv_ << idx << "," << r.ts_p << ",";

            if (r.stereo) {
                if (!raw_s_.is_open())
                    raw_s_.open(dir_ + "/secondary.raw", std::ios::binary);
                raw_s_.write(reinterpret_cast<const char*>(r.right.data()),
                             static_cast<std::streamsize>(r.right.size()));

                // The two camera clocks differ by a constant. This column shows how
                // far that difference has moved since the first pair.
                const int64_t offset = r.ts_s - r.ts_p;
                if (idx == 0) first_offset = offset;
                csv_ << r.ts_s << "," << r.id_p << "," << r.id_s << ","
                     << (offset - first_offset);
            } else {
                csv_ << "," << r.id_p << ",,";  // secondary columns left empty
            }
            csv_ << "\n";
            ++idx;

            // Hand the buffers back for reuse by push().
            {
                std::lock_guard<std::mutex> lk(m_);
                pool_.push_back(std::move(r.left));
                if (r.stereo) pool_.push_back(std::move(r.right));
            }
        }
    }

    std::string dir_;
    size_t max_queued_;
    std::ofstream raw_p_, raw_s_, csv_;
    std::thread worker_;
    std::mutex m_;
    std::condition_variable cond_;
    std::deque<Record> q_;                    // guarded by m_
    std::vector<std::vector<uint8_t>> pool_;  // recycled image buffers, guarded by m_
    bool closed_ = false;                     // guarded by m_

    // Used only by the thread that calls push()
    bool started_ = false;
    bool stereo_ = false;
    size_t bytes_p_ = 0, bytes_s_ = 0;
    uint64_t dropped_ = 0, rejected_ = 0;
};
