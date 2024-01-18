// Copyright 2023, Collabora, Ltd.

#pragma once

#include <vit_interface.h>
#include <vit_implementation_helper.hpp>

#include <cstdint>
#include <memory>
#include <vector>

namespace basalt {

struct TimeStats {
  uint32_t enabled_caps = 0;
  int64_t ts;
  std::vector<int64_t> timings;
  std::vector<std::vector<vit::PoseFeature>> features_per_cam;

  TimeStats(size_t cams) { features_per_cam.reserve(cams); }
  TimeStats() = default;

  void addTime(const std::string_view name, int64_t ts = INT64_MIN) {
    if ((enabled_caps & VIT_TRACKER_POSE_CAPABILITY_TIMING) == 0) {
      return;
    }

    if (timing_titles) {
      const std::string expected(timing_titles[timings.size()]);
      if (expected != name) {
        std::cout << "Invalid timing stage\n";
        std::cout << "expected: " << expected;
        std::cout << ", got: " << name << std::endl;
        exit(EXIT_FAILURE);
      }
    }
    if (ts == INT64_MIN) {
      ts = std::chrono::steady_clock::now().time_since_epoch().count();
    }

    timings.push_back(ts);
  }

  void addFeature(size_t cam, const vit::PoseFeature &feature) {
    if ((enabled_caps & VIT_TRACKER_POSE_CAPABILITY_FEATURES) == 0) {
      return;
    }

    if (cam >= features_per_cam.size()) {
      std::cout << "Invalid camera index\n";
      std::cout << "has: " << features_per_cam.size();
      std::cout << ", got: " << cam << std::endl;
      exit(EXIT_FAILURE);
    }

    features_per_cam.at(cam).push_back(feature);
  }
  const char **timing_titles;
};

struct Tracker final : vit::Tracker {
  Tracker(const vit::Config *config);
  ~Tracker() = default;

  vit::Result has_image_format(const vit::ImageFormat fmt, bool *out_supported) const override;
  vit::Result get_capabilities(vit::TrackerCapability *out_caps) const override;
  vit::Result get_pose_capabilities(vit::TrackerPoseCapability *out_caps) const override;
  vit::Result set_pose_capabilities(const vit::TrackerPoseCapability caps, bool value) override;
  vit::Result start() override;
  vit::Result stop() override;
  vit::Result reset() override;
  vit::Result is_running(bool *running) const override;
  vit::Result add_imu_calibration(const vit::ImuCalibration *calibration) override;
  vit::Result add_camera_calibration(const vit::CameraCalibration *calibration) override;
  vit::Result push_imu_sample(const vit::ImuSample *sample) override;
  vit::Result push_img_sample(const vit::ImgSample *sample) override;
  vit::Result pop_pose(vit::Pose **pose) override;
  vit::Result get_timing_titles(vit::TrackerTimingTitles *out_titles) const override;

 private:
  struct Implementation;
  std::unique_ptr<Implementation> impl;
};

struct Pose final : vit::Pose {
  ~Pose() = default;

  vit::Result get_data(vit::PoseData *out_data) const;
  vit::Result get_timing(vit::PoseTiming *out_timing) const;
  vit::Result get_features(uint32_t camera_index, vit::PoseFeatures *out_features) const;

  struct Implementation;
  std::unique_ptr<Implementation> impl;
};

}  // namespace basalt
