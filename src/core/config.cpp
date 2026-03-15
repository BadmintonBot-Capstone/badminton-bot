#include "core/config.hpp"
#include <yaml-cpp/yaml.h>
#include <opencv2/core.hpp>
#include <stdexcept>
#include <filesystem>

namespace baddy {

static CameraConfig load_camera_config(const std::string& path)
{
    YAML::Node y = YAML::LoadFile(path);
    CameraConfig c;
    if (y["fps"])                        c.fps = y["fps"].as<int>();
    if (y["width"])                      c.width = y["width"].as<int>();
    if (y["height"])                     c.height = y["height"].as<int>();
    if (y["exposure_us"])                c.exposure_us = y["exposure_us"].as<double>();
    if (y["gain_db"])                    c.gain_db = y["gain_db"].as<double>();
    if (y["white_balance_temperature"])  c.white_balance_temperature = y["white_balance_temperature"].as<int>();
    if (y["white_balance_auto"])         c.white_balance_auto = y["white_balance_auto"].as<bool>();
    if (y["pixel_format"])               c.pixel_format = y["pixel_format"].as<std::string>();
    if (y["primary_serial"])             c.primary_serial = y["primary_serial"].as<std::string>();
    if (y["secondary_serial"])           c.secondary_serial = y["secondary_serial"].as<std::string>();
    return c;
}

static DetectionConfig load_detection_config(const std::string& path)
{
    YAML::Node y = YAML::LoadFile(path);
    DetectionConfig d;
    if (y["hsv_low"]) {
        auto v = y["hsv_low"];
        for (int i = 0; i < 3; ++i) d.hsv_low[i] = v[i].as<int>();
    }
    if (y["hsv_high"]) {
        auto v = y["hsv_high"];
        for (int i = 0; i < 3; ++i) d.hsv_high[i] = v[i].as<int>();
    }
    if (y["min_contour_area"])  d.min_contour_area  = y["min_contour_area"].as<int>();
    if (y["morph_kernel_size"]) d.morph_kernel_size = y["morph_kernel_size"].as<int>();
    return d;
}

static EkfConfig load_ekf_config(const std::string& path)
{
    YAML::Node y = YAML::LoadFile(path);
    EkfConfig e;
    if (y["alpha"])                e.alpha = y["alpha"].as<double>();
    if (y["gravity"])              e.gravity = y["gravity"].as<double>();
    if (y["sigma_p"])              e.sigma_p = y["sigma_p"].as<double>();
    if (y["sigma_v"])              e.sigma_v = y["sigma_v"].as<double>();
    if (y["sigma_cam_x"])          e.sigma_cam_x = y["sigma_cam_x"].as<double>();
    if (y["sigma_cam_y"])          e.sigma_cam_y = y["sigma_cam_y"].as<double>();
    if (y["sigma_cam_z"])          e.sigma_cam_z = y["sigma_cam_z"].as<double>();
    if (y["max_coast_frames"])     e.max_coast_frames = y["max_coast_frames"].as<int>();
    if (y["target_intercept_y"])   e.target_intercept_y = y["target_intercept_y"].as<double>();
    if (y["max_predict_time"])     e.max_predict_time = y["max_predict_time"].as<double>();
    if (y["initial_p_position"])   e.initial_p_position = y["initial_p_position"].as<double>();
    if (y["initial_p_velocity"])   e.initial_p_velocity = y["initial_p_velocity"].as<double>();

    if (y["dataset_remap"]) {
        auto r = y["dataset_remap"];
        if (r["x_source"]) e.x_source = r["x_source"].as<std::string>();
        if (r["y_source"]) e.y_source = r["y_source"].as<std::string>();
        if (r["z_source"]) e.z_source = r["z_source"].as<std::string>();
        if (r["x_offset"]) e.x_offset = r["x_offset"].as<double>();
        if (r["y_offset"]) e.y_offset = r["y_offset"].as<double>();
        if (r["z_offset"]) e.z_offset = r["z_offset"].as<double>();
    }
    return e;
}

Config load_config(const std::string& config_dir)
{
    namespace fs = std::filesystem;
    Config cfg;

    auto cam_path = fs::path(config_dir) / "camera.yaml";
    auto det_path = fs::path(config_dir) / "detection.yaml";
    auto ekf_path = fs::path(config_dir) / "ekf.yaml";

    if (fs::exists(cam_path)) cfg.camera    = load_camera_config(cam_path.string());
    if (fs::exists(det_path)) cfg.detection = load_detection_config(det_path.string());
    if (fs::exists(ekf_path)) cfg.ekf       = load_ekf_config(ekf_path.string());

    return cfg;
}

void load_calibration(Config& config, const std::string& calibration_path)
{
    cv::FileStorage fs(calibration_path, cv::FileStorage::READ);
    if (!fs.isOpened())
        throw std::runtime_error("Cannot open calibration file: " + calibration_path);

    fs["M1"] >> config.calibration.M1;
    fs["D1"] >> config.calibration.D1;
    fs["M2"] >> config.calibration.M2;
    fs["D2"] >> config.calibration.D2;
    fs["R1"] >> config.calibration.R1;
    fs["R2"] >> config.calibration.R2;
    fs["P1"] >> config.calibration.P1;
    fs["P2"] >> config.calibration.P2;
    fs["Q"]  >> config.calibration.Q;

    config.calibration.focal_length = config.calibration.P1.at<double>(0, 0);
    config.calibration.baseline = std::abs(config.calibration.P2.at<double>(0, 3))
                                  / config.calibration.focal_length;

    // Court transform is optional — may not exist if calibrate_court.py hasn't been run.
    cv::Mat T_mat;
    fs["T_camera_to_court"] >> T_mat;
    if (!T_mat.empty()) {
        for (int r = 0; r < 4; ++r)
            for (int c = 0; c < 4; ++c)
                config.calibration.T_camera_to_court(r, c) = T_mat.at<double>(r, c);
    }

    config.calibration.loaded = true;
    fs.release();
}

} // namespace baddy
