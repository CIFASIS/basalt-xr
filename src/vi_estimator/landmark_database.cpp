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

#include <algorithm>

#include <basalt/vi_estimator/landmark_database.h>

namespace basalt {

template <class Scalar_>
void LandmarkDatabase<Scalar_>::addLandmark(LandmarkId lm_id, const Landmark<Scalar> &pos) {
  auto &kpt = kpts[lm_id];
  kpt.direction = pos.direction;
  kpt.inv_dist = pos.inv_dist;
  kpt.host_kf_id = pos.host_kf_id;
  kpt.id = lm_id;
  keyframe_obs[pos.host_kf_id].insert(lm_id);
}

template <class Scalar_>
void LandmarkDatabase<Scalar_>::removeFrame(const FrameId &frame) {
  for (auto it = kpts.begin(); it != kpts.end();) {
    for (auto it2 = it->second.obs.begin(); it2 != it->second.obs.end();) {
      if (it2->first.frame_id == frame)
        it2 = removeLandmarkObservationHelper(it, it2);
      else
        it2++;
    }

    if (it->second.obs.size() < min_num_obs) {
      it = removeLandmarkHelper(it);
    } else {
      ++it;
    }
  }
}

template <class Scalar_>
void LandmarkDatabase<Scalar_>::removeKeyframes(const std::set<FrameId> &kfs_to_marg,
                                                const std::set<FrameId> &poses_to_marg,
                                                const std::set<FrameId> &states_to_marg_all) {
  for (auto it = kpts.begin(); it != kpts.end();) {
    if (kfs_to_marg.count(it->second.host_kf_id.frame_id) > 0) {
      it = removeLandmarkHelper(it);
    } else {
      for (auto it2 = it->second.obs.begin(); it2 != it->second.obs.end();) {
        FrameId fid = it2->first.frame_id;
        if (poses_to_marg.count(fid) > 0 || states_to_marg_all.count(fid) > 0 || kfs_to_marg.count(fid) > 0)
          it2 = removeLandmarkObservationHelper(it, it2);
        else
          it2++;
      }

      if (it->second.obs.size() < min_num_obs) {
        it = removeLandmarkHelper(it);
      } else {
        ++it;
      }
    }
  }
  for (const auto &kf_id : kfs_to_marg) {
    keyframe_poses.erase(kf_id);
  }

  std::vector<TimeCamId> tcid_to_rm;
  for (const auto &[tcid, _] : keyframe_obs)
    if (kfs_to_marg.count(tcid.frame_id) > 0) tcid_to_rm.push_back(tcid);

  for (const auto &tcid : tcid_to_rm) keyframe_obs.erase(tcid);
}

template <class Scalar_>
std::vector<TimeCamId> LandmarkDatabase<Scalar_>::getHostKfs() const {
  std::vector<TimeCamId> res;

  for (const auto &kv : observations) res.emplace_back(kv.first);

  return res;
}

template <class Scalar_>
std::vector<const Landmark<Scalar_> *> LandmarkDatabase<Scalar_>::getLandmarksForHost(const TimeCamId &tcid) const {
  std::vector<const Landmark<Scalar> *> res;

  for (const auto &[k, obs] : observations.at(tcid))
    for (const auto &v : obs) res.emplace_back(&kpts.at(v));

  return res;
}

template <class Scalar_>
std::vector<std::pair<LandmarkId, const Landmark<Scalar_> *>> LandmarkDatabase<Scalar_>::getLandmarksForHostWithIds(
    const TimeCamId &tcid) const {
  std::vector<std::pair<LandmarkId, const Landmark<Scalar_> *>> res;

  for (const auto &[k, obs] : observations.at(tcid))
    for (const auto &v : obs) res.emplace_back(v, &kpts.at(v));

  return res;
}

template <class Scalar_>
void LandmarkDatabase<Scalar_>::addObservation(const TimeCamId &tcid_target, const KeypointObservation<Scalar> &o) {
  auto it = kpts.find(o.kpt_id);
  BASALT_ASSERT(it != kpts.end());

  it->second.obs[tcid_target] = o.pos;

  observations[it->second.host_kf_id][tcid_target].insert(it->first);

  keyframe_obs[tcid_target].insert(it->first);
}

template <class Scalar_>
void LandmarkDatabase<Scalar_>::addKeyframe(int64_t kf_id, const SE3 &pos) {
  keyframe_poses[kf_id] = pos;
}

template <class Scalar_>
Sophus::SE3<Scalar_> &LandmarkDatabase<Scalar_>::getKeyframePose(int64_t kf_id) {
  return keyframe_poses.at(kf_id);
}


template <class Scalar_>
void LandmarkDatabase<Scalar_>::mergeLMDB(LandmarkDatabase<Scalar>::Ptr lmdb, bool override) {
  // Add keyframes
  for (const auto &[kf_id, pose] : lmdb->getKeyframes()) {
    if (!override && keyframeExists(kf_id)) continue;  // Skip if the landmark already exists
    addKeyframe(kf_id, pose);
  }

  // Add Landmarks
  for (const auto &[lm_id, lm] : lmdb->getLandmarks()) {
    if (override || !landmarkExists(lm_id)) addLandmark(lm_id, lm);

    // Add Observations
    for (const auto &[tcid_target, pos] : lm.obs) {
      if (!keyframeExists(tcid_target.frame_id) && !lmdb->keyframeExists(tcid_target.frame_id))
        continue;  // Basalt adds observations to the LMDB before the frames are keyframes..

      if (!override && observationExists(tcid_target, lm_id)) continue;

      KeypointObservation<Scalar> kobs;
      kobs.kpt_id = lm_id;
      kobs.pos = pos;
      addObservation(tcid_target, kobs);
    }

    // Remove the landmark if it has less than min_num_obs observations between both lmdb.
    // This check must be here because not all the observations are added the current lmdb
    if (numObservations(lm_id) < min_num_obs) {
      removeLandmark(lm_id);
      continue;
    }
  }
}

template <class Scalar_>
Landmark<Scalar_> &LandmarkDatabase<Scalar_>::getLandmark(LandmarkId lm_id) {
  return kpts.at(lm_id);
}

template <class Scalar_>
const Landmark<Scalar_> &LandmarkDatabase<Scalar_>::getLandmark(LandmarkId lm_id) const {
  return kpts.at(lm_id);
}

template <class Scalar_>
const std::unordered_map<TimeCamId, std::map<TimeCamId, std::set<LandmarkId>>> &
LandmarkDatabase<Scalar_>::getObservations() const {
  return observations;
}

template <class Scalar_>
const Eigen::aligned_unordered_map<LandmarkId, Landmark<Scalar_>> &LandmarkDatabase<Scalar_>::getLandmarks() const {
  return kpts;
}

template <class Scalar_>
const Eigen::aligned_map<FrameId, Sophus::SE3<Scalar_>> &LandmarkDatabase<Scalar_>::getKeyframes() const {
  if (keyframe_poses.empty()) {
    static const Eigen::aligned_map<FrameId, Sophus::SE3<Scalar_>> empty_map{};
    return empty_map;
  }
  return keyframe_poses;
}

template <class Scalar_>
const Eigen::aligned_map<TimeCamId, std::set<LandmarkId>> &LandmarkDatabase<Scalar_>::getKeyframeObs() const {
  return keyframe_obs;
}

template <class Scalar_>
bool LandmarkDatabase<Scalar_>::landmarkExists(int lm_id) const {
  return kpts.count(lm_id) > 0;
}

template <class Scalar_>
bool LandmarkDatabase<Scalar_>::keyframeExists(FrameId kf_id) const {
  return keyframe_poses.count(kf_id) > 0;
}

template <class Scalar_>
bool LandmarkDatabase<Scalar_>::observationExists(TimeCamId target_tcid, LandmarkId lm_id) {
  // Check if the observation (host, target) -> lm exists
  if (!landmarkExists(lm_id)) return false;
  auto lm = getLandmark(lm_id);

  auto it1 = observations.find(lm.host_kf_id);
  if (it1 == observations.end()) return false;

  auto it2 = it1->second.find(target_tcid);
  if (it2 == it1->second.end()) return false;

  return it2->second.count(lm_id) > 0;
}

template <class Scalar_>
size_t LandmarkDatabase<Scalar_>::numLandmarks() const {
  return kpts.size();
}

template <class Scalar_>
int LandmarkDatabase<Scalar_>::numObservations() const {
  int num_observations = 0;

  for (const auto &[_, val_map] : observations) {
    for (const auto &[_, val] : val_map) {
      num_observations += val.size();
    }
  }

  return num_observations;
}

template <class Scalar_>
int LandmarkDatabase<Scalar_>::numObservations(LandmarkId lm_id) const {
  return kpts.at(lm_id).obs.size();
}

template <class Scalar_>
int LandmarkDatabase<Scalar_>::numKeyframes() const {
  return keyframe_poses.size();
}

template <class Scalar_>
typename LandmarkDatabase<Scalar_>::MapIter LandmarkDatabase<Scalar_>::removeLandmarkHelper(
    LandmarkDatabase<Scalar>::MapIter it) {
  auto host_it = observations.find(it->second.host_kf_id);

  if (host_it != observations.end()) {
    for (const auto &[k, v] : it->second.obs) {
      auto target_it = host_it->second.find(k);
      target_it->second.erase(it->first);

      if (target_it->second.empty()) host_it->second.erase(target_it);
    }

    if (host_it->second.empty()) observations.erase(host_it);
  }

  for (const auto &[tcid, obs] : keyframe_obs) {
    if (obs.count(it->first) > 0) keyframe_obs[tcid].erase(it->first);
  }

  return kpts.erase(it);
}

template <class Scalar_>
typename Landmark<Scalar_>::MapIter LandmarkDatabase<Scalar_>::removeLandmarkObservationHelper(
    LandmarkDatabase<Scalar>::MapIter it, typename Landmark<Scalar>::MapIter it2) {
  auto host_it = observations.find(it->second.host_kf_id);
  auto target_it = host_it->second.find(it2->first);
  target_it->second.erase(it->first);

  if (target_it->second.empty()) host_it->second.erase(target_it);
  if (host_it->second.empty()) observations.erase(host_it);

  keyframe_obs.find(it2->first)->second.erase(it->first);

  return it->second.obs.erase(it2);
}

template <class Scalar_>
void LandmarkDatabase<Scalar_>::removeLandmark(LandmarkId lm_id) {
  auto it = kpts.find(lm_id);
  if (it != kpts.end()) removeLandmarkHelper(it);
}

template <class Scalar_>
void LandmarkDatabase<Scalar_>::removeObservations(LandmarkId lm_id, const std::set<TimeCamId> &obs) {
  auto it = kpts.find(lm_id);
  BASALT_ASSERT(it != kpts.end());

  for (auto it2 = it->second.obs.begin(); it2 != it->second.obs.end();) {
    if (obs.count(it2->first) > 0) {
      it2 = removeLandmarkObservationHelper(it, it2);
    } else
      it2++;
  }

  if (it->second.obs.size() < min_num_obs) {
    removeLandmarkHelper(it);
  }
}

// //////////////////////////////////////////////////////////////////
// instatiate templates

// Note: double specialization is unconditional, b/c NfrMapper depends on it.
// #ifdef BASALT_INSTANTIATIONS_DOUBLE
template class LandmarkDatabase<double>;
// #endif

#ifdef BASALT_INSTANTIATIONS_FLOAT
template class LandmarkDatabase<float>;
#endif

}  // namespace basalt
