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

#include <basalt/optical_flow/optical_flow.h>
#include <basalt/utils/imu_types.h>
#include <basalt/utils/eigen_utils.hpp>

namespace basalt {

template <class Scalar_>
struct KeypointObservation {
  using Scalar = Scalar_;

  int kpt_id;
  Eigen::Matrix<Scalar, 2, 1> pos;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// keypoint position defined relative to some frame
template <class Scalar_>
struct Landmark {
  using Scalar = Scalar_;
  using Vec2 = Eigen::Matrix<Scalar, 2, 1>;

  using ObsMap = Eigen::aligned_map<TimeCamId, Vec2>;
  using MapIter = typename ObsMap::iterator;

  // 3D position parameters
  Vec2 direction;
  Scalar inv_dist;

  // Observations
  TimeCamId host_kf_id;
  ObsMap obs;

  LandmarkId id;

  inline void backup() {
    backup_direction = direction;
    backup_inv_dist = inv_dist;
  }

  inline void restore() {
    direction = backup_direction;
    inv_dist = backup_inv_dist;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  Vec2 backup_direction;
  Scalar backup_inv_dist;
};

template <class Scalar_>
class LandmarkDatabase {
 public:
  using Scalar = Scalar_;
  using SE3 = Sophus::SE3<Scalar>;

  typedef std::shared_ptr<LandmarkDatabase<Scalar>> Ptr;

  LandmarkDatabase(std::string name = "Landmark Database") : debug_name(name){};

  // Non-const
  void addLandmark(LandmarkId lm_id, const Landmark<Scalar>& pos);

  void removeFrame(const FrameId& frame);

  void clear() {
    kpts.clear();
    observations.clear();
    keyframe_idx.clear();
    keyframe_poses.clear();
    keyframe_obs.clear();
  }

  void removeKeyframes(const std::set<FrameId>& kfs_to_marg, const std::set<FrameId>& poses_to_marg,
                       const std::set<FrameId>& states_to_marg_all);

  void addObservation(const TimeCamId& tcid_target, const KeypointObservation<Scalar>& o);

  Landmark<Scalar>& getLandmark(LandmarkId lm_id);

  void addKeyframe(FrameId kf_id, size_t index, const SE3& pos);

  SE3& getKeyframePose(FrameId kf_id);

  size_t& getKeyframeIndex(FrameId kf_id);

  void mergeLMDB(LandmarkDatabase<Scalar>::Ptr lmdb, bool override);

  // Const
  const Landmark<Scalar>& getLandmark(LandmarkId lm_id) const;

  std::vector<TimeCamId> getHostKfs() const;

  std::vector<const Landmark<Scalar>*> getLandmarksForHost(const TimeCamId& tcid) const;

  std::vector<std::pair<LandmarkId, const Landmark<Scalar>*>> getLandmarksForHostWithIds(const TimeCamId& tcid) const;

  const std::unordered_map<TimeCamId, std::map<TimeCamId, std::set<LandmarkId>>>& getObservations() const;

  const Eigen::aligned_unordered_map<LandmarkId, Landmark<Scalar>>& getLandmarks() const;

  const Eigen::aligned_map<FrameId, SE3>& getKeyframes() const;

  const Eigen::aligned_map<TimeCamId, std::set<LandmarkId>>& getKeyframeObs() const;

  bool landmarkExists(int lm_id) const;

  bool keyframeExists(FrameId kf_id) const;

  bool observationExists(TimeCamId target_tcid, LandmarkId lm_id);

  size_t numLandmarks() const;

  int numObservations() const;

  int numObservations(LandmarkId lm_id) const;

  int numKeyframes() const;

  void removeLandmark(LandmarkId lm_id);

  void removeObservations(LandmarkId lm_id, const std::set<TimeCamId>& obs);

  void print(bool show_ids = false);

  inline void backup() {
    for (auto& kv : kpts) kv.second.backup();
  }

  inline void restore() {
    for (auto& kv : kpts) kv.second.restore();
  }

 private:
  using MapIter = typename Eigen::aligned_unordered_map<LandmarkId, Landmark<Scalar>>::iterator;
  MapIter removeLandmarkHelper(MapIter it);
  typename Landmark<Scalar>::MapIter removeLandmarkObservationHelper(MapIter it,
                                                                     typename Landmark<Scalar>::MapIter it2);

  Eigen::aligned_unordered_map<LandmarkId, Landmark<Scalar>> kpts;

  std::unordered_map<TimeCamId, std::map<TimeCamId, std::set<LandmarkId>>> observations;

  Eigen::aligned_map<FrameId, size_t> keyframe_idx;

  Eigen::aligned_map<FrameId, SE3> keyframe_poses;

  Eigen::aligned_map<TimeCamId, std::set<LandmarkId>> keyframe_obs;

  static constexpr int min_num_obs = 2;

  std::string debug_name;
};

}  // namespace basalt
