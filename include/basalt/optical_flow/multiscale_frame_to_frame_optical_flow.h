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

// Original source for multi-scale implementation:
// https://github.com/DLR-RM/granite (MIT license)

#pragma once

#include <sophus/se2.hpp>

#include <tbb/blocked_range.h>
#include <tbb/concurrent_unordered_map.h>
#include <tbb/parallel_for.h>

#include <basalt/optical_flow/optical_flow.h>
#include <basalt/optical_flow/patch.h>

#include <basalt/utils/keypoints.h>

namespace basalt {

/// MultiscaleFrameToFrameOpticalFlow is the same as FrameToFrameOpticalFlow,
/// but patches can be created at all pyramid levels, not just the lowest
/// pyramid.
template <typename Scalar, template <typename> typename Pattern>
class MultiscaleFrameToFrameOpticalFlow : public OpticalFlowTyped<Scalar, Pattern> {
 public:
  using typename OpticalFlowTyped<Scalar, Pattern>::PatchT;
  using typename OpticalFlowTyped<Scalar, Pattern>::Vector2;
  using typename OpticalFlowTyped<Scalar, Pattern>::Matrix2;
  using typename OpticalFlowTyped<Scalar, Pattern>::Vector3;
  using typename OpticalFlowTyped<Scalar, Pattern>::Matrix3;
  using typename OpticalFlowTyped<Scalar, Pattern>::Vector4;
  using typename OpticalFlowTyped<Scalar, Pattern>::Matrix4;
  using typename OpticalFlowTyped<Scalar, Pattern>::SE2;
  using typename OpticalFlowTyped<Scalar, Pattern>::SE3;
  using OpticalFlowTyped<Scalar, Pattern>::getNumCams;
  using OpticalFlowTyped<Scalar, Pattern>::calib;
  using OpticalFlowTyped<Scalar, Pattern>::E;

  using OpticalFlowBase::config;
  using OpticalFlowBase::depth_guess;
  using OpticalFlowBase::first_state_arrived;
  using OpticalFlowBase::frame_counter;
  using OpticalFlowBase::input_depth_queue;
  using OpticalFlowBase::input_img_queue;
  using OpticalFlowBase::input_imu_queue;
  using OpticalFlowBase::input_state_queue;
  using OpticalFlowBase::last_keypoint_id;
  using OpticalFlowBase::latest_state;
  using OpticalFlowBase::old_pyramid;
  using OpticalFlowBase::output_queue;
  using OpticalFlowBase::patch_coord;
  using OpticalFlowBase::predicted_state;
  using OpticalFlowBase::processing_thread;
  using OpticalFlowBase::pyramid;
  using OpticalFlowBase::show_gui;
  using OpticalFlowBase::t_ns;
  using OpticalFlowBase::transforms;

  MultiscaleFrameToFrameOpticalFlow(const VioConfig& conf, const Calibration<double>& cal)
      : OpticalFlowTyped<Scalar, Pattern>(conf, cal) {}

  void processingLoop() override {
    OpticalFlowInput::Ptr img;

    while (true) {
      while (input_depth_queue.try_pop(depth_guess)) continue;

      input_img_queue.pop(img);

      if (!img.get()) {
        if (output_queue) output_queue->push(nullptr);
        break;
      }
      img->addTime("frames_received");

      processFrame(img->t_ns, img);
    }
  }

  bool processFrame(int64_t curr_t_ns, OpticalFlowInput::Ptr& new_img_vec) {
    for (const auto& v : new_img_vec->img_data) {
      if (!v.img.get()) {
        std::cout << "Image for " << curr_t_ns << " not present!" << std::endl;
        return true;
      }
    }

    const size_t num_cams = getNumCams();

    if (t_ns < 0) {
      t_ns = curr_t_ns;

      transforms.reset(new OpticalFlowResult);
      transforms->observations.resize(num_cams);
      transforms->pyramid_levels.resize(num_cams);
      transforms->t_ns = t_ns;

      pyramid.reset(new std::vector<ManagedImagePyr<uint16_t>>);
      pyramid->resize(num_cams);

      tbb::parallel_for(tbb::blocked_range<size_t>(0, num_cams), [&](const tbb::blocked_range<size_t>& r) {
        for (size_t i = r.begin(); i != r.end(); ++i) {
          pyramid->at(i).setFromImage(*new_img_vec->img_data[i].img, config.optical_flow_levels);
        }
      });

      transforms->input_images = new_img_vec;

      addPoints();
      filterPoints();
    } else {
      t_ns = curr_t_ns;

      old_pyramid = pyramid;

      pyramid.reset(new std::vector<ManagedImagePyr<uint16_t>>);
      pyramid->resize(num_cams);
      tbb::parallel_for(tbb::blocked_range<size_t>(0, num_cams), [&](const tbb::blocked_range<size_t>& r) {
        for (size_t i = r.begin(); i != r.end(); ++i) {
          pyramid->at(i).setFromImage(*new_img_vec->img_data[i].img, config.optical_flow_levels);
        }
      });

      OpticalFlowResult::Ptr new_transforms;
      new_transforms.reset(new OpticalFlowResult);
      new_transforms->observations.resize(num_cams);
      new_transforms->pyramid_levels.resize(num_cams);
      new_transforms->t_ns = t_ns;

      for (size_t i = 0; i < num_cams; i++) {
        trackPoints(old_pyramid->at(i), pyramid->at(i), transforms->observations[i], transforms->pyramid_levels[i],
                    new_transforms->observations[i], new_transforms->pyramid_levels[i], i, i);
      }

      // std::cout << t_ns << ": Could track "
      //           << new_transforms->observations.at(0).size() << " points."
      //           << std::endl;

      transforms = new_transforms;
      transforms->input_images = new_img_vec;

      addPoints();
      filterPoints();
    }

    if (frame_counter % config.optical_flow_skip_frames == 0) {
      transforms->input_images->addTime("opticalflow_produced");
      try {
        output_queue->push(transforms);
      } catch (const tbb::user_abort&) {
        return false;
      };
    }

    frame_counter++;
    return true;
  }

  void trackPoints(const ManagedImagePyr<uint16_t>& pyr_1, const ManagedImagePyr<uint16_t>& pyr_2,
                   const Keypoints& transform_map_1, const std::map<KeypointId, size_t>& pyramid_levels_1,
                   Keypoints& transform_map_2, std::map<KeypointId, size_t>& pyramid_levels_2,  //
                   size_t cam1, size_t cam2) const {
    size_t num_points = transform_map_1.size();

    std::vector<KeypointId> ids;
    Eigen::aligned_vector<Eigen::AffineCompact2f> init_vec;
    std::vector<size_t> pyramid_level;

    ids.reserve(num_points);
    init_vec.reserve(num_points);
    pyramid_level.reserve(num_points);

    for (const auto& kv : transform_map_1) {
      ids.push_back(kv.first);
      init_vec.push_back(kv.second);
      pyramid_level.push_back(pyramid_levels_1.at(kv.first));
    }

    tbb::concurrent_unordered_map<KeypointId, Eigen::AffineCompact2f, std::hash<KeypointId>> result_transforms;
    tbb::concurrent_unordered_map<KeypointId, size_t, std::hash<KeypointId>> result_pyramid_level;

    double depth = depth_guess;
    transforms->input_images->depth_guess = depth;  // Store guess for UI

    bool matching = cam1 != cam2;
    MatchingGuessType guess_type = config.optical_flow_matching_guess_type;
    bool guess_requires_depth = guess_type != MatchingGuessType::SAME_PIXEL;
    const bool use_depth = matching && guess_requires_depth;

    auto compute_func = [&](const tbb::blocked_range<size_t>& range) {
      for (size_t r = range.begin(); r != range.end(); ++r) {
        const KeypointId id = ids[r];

        const Eigen::AffineCompact2f& transform_1 = init_vec[r];
        Eigen::AffineCompact2f transform_2 = transform_1;

        auto t1 = transform_1.translation();
        auto t2 = transform_2.translation();

        Eigen::Vector2f off{0, 0};
        if (use_depth) {
          off = calib.viewOffset(t1, depth, cam1, cam2);
        }

        t2 -= off;  // This modifies transform_2

        bool valid = t2(0) >= 0 && t2(1) >= 0 && t2(0) < pyr_2.lvl(0).w && t2(1) < pyr_2.lvl(0).h;
        if (!valid) continue;

        valid = trackPoint(pyr_1, pyr_2, transform_1, pyramid_level[r], transform_2);
        if (!valid) continue;

        Eigen::AffineCompact2f transform_1_recovered = transform_2;
        auto t1_recovered = transform_1_recovered.translation();

        t1_recovered += off;

        valid = trackPoint(pyr_2, pyr_1, transform_2, pyramid_level[r], transform_1_recovered);
        if (!valid) continue;

        const Scalar scale = 1 << pyramid_level[r];
        Scalar dist2 = ((t1 - t1_recovered) / scale).squaredNorm();

        if (dist2 < config.optical_flow_max_recovered_dist2) {
          result_transforms[id] = transform_2;
          result_pyramid_level[id] = pyramid_level[r];
        }
      }
    };

    tbb::blocked_range<size_t> range(0, num_points);

    tbb::parallel_for(range, compute_func);
    // compute_func(range);

    transform_map_2.clear();
    transform_map_2.insert(result_transforms.begin(), result_transforms.end());
    pyramid_levels_2.clear();
    pyramid_levels_2.insert(result_pyramid_level.begin(), result_pyramid_level.end());
  }

  inline bool trackPoint(const ManagedImagePyr<uint16_t>& old_pyr, const ManagedImagePyr<uint16_t>& pyr,
                         const Eigen::AffineCompact2f& old_transform, const size_t pyramid_level,
                         Eigen::AffineCompact2f& transform) const {
    bool patch_valid = true;

    transform.linear().setIdentity();

    for (ssize_t level = config.optical_flow_levels; level >= static_cast<ssize_t>(pyramid_level); level--) {
      const Scalar scale = 1 << level;

      Eigen::AffineCompact2f transform_tmp = transform;

      transform_tmp.translation() /= scale;

      PatchT p(old_pyr.lvl(level), old_transform.translation() / scale);

      patch_valid &= p.valid;
      if (patch_valid) {
        // Perform tracking on current level
        patch_valid &= trackPointAtLevel(pyr.lvl(level), p, transform_tmp);
      }

      if (level == static_cast<ssize_t>(pyramid_level) + 1 && !patch_valid) {
        return false;
      }

      transform_tmp.translation() *= scale;

      if (patch_valid) {
        transform = transform_tmp;
      }
    }

    transform.linear() = old_transform.linear() * transform.linear();

    return patch_valid;
  }

  inline bool trackPointAtLevel(const Image<const uint16_t>& img_2, const PatchT& dp,
                                Eigen::AffineCompact2f& transform) const {
    bool patch_valid = true;

    for (int iteration = 0; patch_valid && iteration < config.optical_flow_max_iterations; iteration++) {
      typename PatchT::VectorP res;

      typename PatchT::Matrix2P transformed_pat = transform.linear().matrix() * PatchT::pattern2;
      transformed_pat.colwise() += transform.translation();

      patch_valid &= dp.residual(img_2, transformed_pat, res);

      if (patch_valid) {
        const Vector3 inc = -dp.H_se2_inv_J_se2_T * res;

        // avoid NaN in increment (leads to SE2::exp crashing)
        patch_valid &= inc.array().isFinite().all();

        // avoid very large increment
        patch_valid &= inc.template lpNorm<Eigen::Infinity>() < 1e6;

        if (patch_valid) {
          transform *= SE2::exp(inc).matrix();

          const int filter_margin = 2;

          patch_valid &= img_2.InBounds(transform.translation(), filter_margin);
        }
      }
    }

    return patch_valid;
  }

  void addPoints() {
    KeypointsData kd;

    Keypoints new_poses_main, new_poses_stereo;
    std::map<KeypointId, size_t> new_pyramid_levels_main, new_pyramid_levels_stereo;

    for (ssize_t level = 0; level < static_cast<ssize_t>(config.optical_flow_levels) - 1; level++) {
      Eigen::aligned_vector<Eigen::Vector2d> pts;

      for (const auto& kv : transforms->observations.at(0)) {
        const ssize_t point_level = transforms->pyramid_levels.at(0).at(kv.first);

        // do not create points were already points at similar levels are
        if (point_level <= level + 1 && point_level >= level - 1) {
          // if (point_level == level) {
          const Scalar scale = 1 << point_level;
          pts.emplace_back((kv.second.translation() / scale).template cast<double>());
        }
      }

      detectKeypoints(pyramid->at(0).lvl(level), kd, config.optical_flow_detection_grid_size,
                      config.optical_flow_detection_num_points_cell, config.optical_flow_detection_min_threshold,
                      config.optical_flow_detection_max_threshold, transforms->input_images->masks.at(0), pts);

      const Scalar scale = 1 << level;

      for (size_t i = 0; i < kd.corners.size(); i++) {
        Eigen::AffineCompact2f transform;
        transform.setIdentity();
        transform.translation() = kd.corners[i].cast<Scalar>() * scale;  // TODO cast float?

        transforms->observations.at(0)[last_keypoint_id] = transform;
        transforms->pyramid_levels.at(0)[last_keypoint_id] = level;
        new_poses_main[last_keypoint_id] = transform;
        new_pyramid_levels_main[last_keypoint_id] = level;

        last_keypoint_id++;
      }

      trackPoints(pyramid->at(0), pyramid->at(1), new_poses_main, new_pyramid_levels_main, new_poses_stereo,
                  new_pyramid_levels_stereo, 0, 1);

      for (const auto& kv : new_poses_stereo) {
        transforms->observations.at(1).emplace(kv);
        transforms->pyramid_levels.at(1)[kv.first] = new_pyramid_levels_stereo.at(kv.first);
      }
    }
  }

  void filterPoints() {
    std::set<KeypointId> lm_to_remove;

    std::vector<KeypointId> kpid;
    Eigen::aligned_vector<Eigen::Vector2f> proj0, proj1;

    for (const auto& kv : transforms->observations.at(1)) {
      auto it = transforms->observations.at(0).find(kv.first);

      if (it != transforms->observations.at(0).end()) {
        proj0.emplace_back(it->second.translation());
        proj1.emplace_back(kv.second.translation());
        kpid.emplace_back(kv.first);
      }
    }

    Eigen::aligned_vector<Eigen::Vector4f> p3d_main, p3d_stereo;
    std::vector<bool> p3d_main_success, p3d_stereo_success;

    calib.intrinsics[0].unproject(proj0, p3d_main, p3d_main_success);
    calib.intrinsics[1].unproject(proj1, p3d_stereo, p3d_stereo_success);

    for (size_t i = 0; i < p3d_main_success.size(); i++) {
      if (p3d_main_success[i] && p3d_stereo_success[i]) {
        const double epipolar_error = std::abs(p3d_main[i].transpose() * E[1] * p3d_stereo[i]);

        const Scalar scale = 1 << transforms->pyramid_levels.at(0).at(kpid[i]);

        if (epipolar_error > config.optical_flow_epipolar_error * scale) {
          lm_to_remove.emplace(kpid[i]);
        }
      } else {
        lm_to_remove.emplace(kpid[i]);
      }
    }

    for (int id : lm_to_remove) {
      transforms->observations.at(1).erase(id);
    }
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:
};

}  // namespace basalt
