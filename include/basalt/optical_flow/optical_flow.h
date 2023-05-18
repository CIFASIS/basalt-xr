/**
BSD 3-Clause License

This file is part of the Basalt project.
https://gitlab.com/VladyslavUsenko/basalt.git

Copyright (c) 2019, Vladyslav Usenko and Nikolaus Demmel.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#pragma once

#include <chrono>
#include <memory>

#include <Eigen/Geometry>

#include <basalt/utils/vio_config.h>

#include <basalt/imu/imu_types.h>
#include <basalt/io/dataset_io.h>
#include <basalt/utils/keypoints.h>
#include <basalt/calibration/calibration.hpp>
#include <basalt/camera/stereographic_param.hpp>
#include <basalt/utils/sophus_utils.hpp>
#include <slam_tracker.hpp>
#include <utility>
#include "sophus/se3.hpp"

#include <tbb/concurrent_queue.h>

namespace basalt {

using KeypointId = size_t;
// TODO: Unify Keypoints + Descriptors
using Keypoints = Eigen::aligned_map<KeypointId, Eigen::AffineCompact2f>;
using Descriptor = std::bitset<256>;
using Descriptors = Eigen::aligned_map<KeypointId, Descriptor>;
using xrt::auxiliary::tracking::slam::timestats;

struct OpticalFlowInput {
  using Ptr = std::shared_ptr<OpticalFlowInput>;
  using Vec3 = Eigen::Matrix<double, 3, 1>;

  OpticalFlowInput() = default;

  OpticalFlowInput(int NUM_CAMS) {
    img_data.resize(NUM_CAMS);
    masks.resize(NUM_CAMS);
  }

  int64_t t_ns;
  std::vector<ImageData> img_data;

  // Recorded internal pipeline values for UI playback
  double depth_guess = -1;

  std::vector<Masks> masks;  //!< Regions of the image to ignore

  timestats stats;  //!< Keeps track of internal metrics for this t_ns
  void addTime(const char* name, int64_t custom_ts = INT64_MIN) {
    stats.addTime(name, custom_ts);
  }
};

struct OpticalFlowResult {
  using Ptr = std::shared_ptr<OpticalFlowResult>;

  int64_t t_ns;
  // TODO: Unify obs + desc in same struct
  std::vector<Keypoints> observations;
  std::vector<Descriptors> descriptors;
  std::vector<Keypoints> tracking_guesses;
  std::vector<Keypoints> matching_guesses;

  std::vector<std::map<KeypointId, size_t>> pyramid_levels;

  OpticalFlowInput::Ptr input_images;
};

class OpticalFlowBase {
 public:
  using Ptr = std::shared_ptr<OpticalFlowBase>;

  tbb::concurrent_bounded_queue<OpticalFlowInput::Ptr> input_queue;
  tbb::concurrent_bounded_queue<ImuData<double>::Ptr> input_imu_queue;
  tbb::concurrent_queue<double> input_depth_queue;
  tbb::concurrent_queue<PoseVelBiasState<double>::Ptr> input_state_queue;
  tbb::concurrent_bounded_queue<OpticalFlowResult::Ptr>* output_queue = nullptr;

  Eigen::MatrixXf patch_coord;
  double depth_guess = -1;
  PoseVelBiasState<double>::Ptr latest_state = nullptr;
  PoseVelBiasState<double>::Ptr predicted_state = nullptr;

  bool first_state_arrived = false;
  bool show_gui;  //!< Whether we need to store additional info for the UI
};

class OpticalFlowFactory {
 public:
  static OpticalFlowBase::Ptr getOpticalFlow(const VioConfig& config,
                                             const Calibration<double>& cam);
};
}  // namespace basalt
