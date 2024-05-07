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

#include <basalt/vi_estimator/marg_helper.h>
#include <basalt/vi_estimator/sqrt_keypoint_vio.h>

#include <basalt/optical_flow/optical_flow.h>
#include <basalt/optimization/accumulator.h>
#include <basalt/utils/assert.h>
#include <basalt/utils/system_utils.h>
#include <basalt/vi_estimator/sc_ba_base.h>
#include <basalt/utils/cast_utils.hpp>
#include <basalt/utils/format.hpp>
#include <basalt/utils/time_utils.hpp>

#include <basalt/linearization/linearization_base.hpp>

#include <fmt/format.h>

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_reduce.h>

#include <chrono>

namespace basalt {

template <class Scalar_>
SqrtKeypointVioEstimator<Scalar_>::SqrtKeypointVioEstimator(const Eigen::Vector3d& g_,
                                                            const basalt::Calibration<double>& calib_,
                                                            const VioConfig& config_)
    : take_kf(true),
      frames_after_kf(0),
      g(g_.cast<Scalar>()),
      initialized(false),
      schedule_reset(false),
      config(config_),
      lambda(config_.vio_lm_lambda_initial),
      min_lambda(config_.vio_lm_lambda_min),
      max_lambda(config_.vio_lm_lambda_max),
      lambda_vee(2) {
  obs_std_dev = Scalar(config.vio_obs_std_dev);
  huber_thresh = Scalar(config.vio_obs_huber_thresh);
  calib = calib_.cast<Scalar>();

  // Setup marginalization
  marg_data.is_sqrt = config.vio_sqrt_marg;
  marg_data.H.setZero(POSE_VEL_BIAS_SIZE, POSE_VEL_BIAS_SIZE);
  marg_data.b.setZero(POSE_VEL_BIAS_SIZE);

  // Version without prior
  nullspace_marg_data.is_sqrt = marg_data.is_sqrt;
  nullspace_marg_data.H.setZero(POSE_VEL_BIAS_SIZE, POSE_VEL_BIAS_SIZE);
  nullspace_marg_data.b.setZero(POSE_VEL_BIAS_SIZE);

  if (marg_data.is_sqrt) {
    // prior on position
    marg_data.H.diagonal().template head<3>().setConstant(std::sqrt(Scalar(config.vio_init_pose_weight)));
    // prior on yaw
    marg_data.H(5, 5) = std::sqrt(Scalar(config.vio_init_pose_weight));

    // small prior to avoid jumps in bias
    marg_data.H.diagonal().template segment<3>(9).array() = std::sqrt(Scalar(config.vio_init_ba_weight));
    marg_data.H.diagonal().template segment<3>(12).array() = std::sqrt(Scalar(config.vio_init_bg_weight));
  } else {
    // prior on position
    marg_data.H.diagonal().template head<3>().setConstant(Scalar(config.vio_init_pose_weight));
    // prior on yaw
    marg_data.H(5, 5) = Scalar(config.vio_init_pose_weight);

    // small prior to avoid jumps in bias
    marg_data.H.diagonal().template segment<3>(9).array() = Scalar(config.vio_init_ba_weight);
    marg_data.H.diagonal().template segment<3>(12).array() = Scalar(config.vio_init_bg_weight);
  }

  std::cout << "marg_H (sqrt:" << marg_data.is_sqrt << ")\n" << marg_data.H << std::endl;

  gyro_bias_sqrt_weight = calib.gyro_bias_std.array().inverse();
  accel_bias_sqrt_weight = calib.accel_bias_std.array().inverse();

  max_states = config.vio_max_states;
  max_kfs = config.vio_max_kfs;

  opt_started = false;

  vision_data_queue.set_capacity(10);
  imu_data_queue.set_capacity(300);

  lmdb = LandmarkDatabase<Scalar>("VIO LMDB");
}

template <class Scalar>
void SqrtKeypointVioEstimator<Scalar>::scheduleResetState() {
  schedule_reset = true;
}

template <class Scalar>
void SqrtKeypointVioEstimator<Scalar>::takeLongTermKeyframe() {
  take_ltkf = true;
}

template <class Scalar>
void SqrtKeypointVioEstimator<Scalar>::GetCovisibilityMap() {
  get_map = true;
}

template <class Scalar>
void SqrtKeypointVioEstimator<Scalar>::addZeroKeyframeToMargData(FrameId toadd_ts) {
  // Update marg_data for a timestamped keyframe so that H and b add corresponding zeroed indices

  size_t total_rows = marg_data.H.rows();
  size_t old_total_cols = marg_data.H.cols();
  size_t new_total_cols = old_total_cols + POSE_SIZE;
  MatX Hm;
  Hm.setZero(total_rows, new_total_cols);

  AbsOrderMap order{};
  size_t added_idx = SIZE_MAX;
  for (const auto& [ts, idxsize] : marg_data.order.abs_order_map) {
    if (added_idx == SIZE_MAX && toadd_ts < ts) {
      added_idx = order.total_size;
      order.abs_order_map[toadd_ts] = std::make_pair(added_idx, POSE_SIZE);
      order.total_size += POSE_SIZE;
      order.items++;
    }

    const auto& [idx, size] = idxsize;
    order.abs_order_map[ts] = std::make_pair(order.total_size, size);
    order.total_size += size;
    order.items++;
  }

  if (added_idx == SIZE_MAX) {  // In the unusual case the frame to add is in the future
    added_idx = order.total_size;
    order.abs_order_map[toadd_ts] = std::make_pair(added_idx, POSE_SIZE);
    order.total_size += POSE_SIZE;
    order.items++;
  }

  size_t bef = added_idx;                   // Side size to the left (before)
  size_t aft = old_total_cols - added_idx;  // Cols after idx (after)
  size_t i = added_idx;                     // Index to insert at
  size_t s = POSE_SIZE;                     // Number of inserted vars

  if (i == 0) {                                                                                         // If at the beginning
    Hm.template block(0, i + s, total_rows, aft) = marg_data.H.template block(0, i, total_rows, aft);   // right
  } else if (i + s == order.total_size) {                                                               // If at the end
    Hm.template block(0, 0, total_rows, bef) = marg_data.H.template block(0, 0, total_rows, bef);       // left
  } else {                                                                                              // If in the middle
    Hm.template block(0, 0, total_rows, bef) = marg_data.H.template block(0, 0, total_rows, bef);       // left
    Hm.template block(0, i + s, total_rows, aft) = marg_data.H.template block(0, i, total_rows, aft);   // right
  }

  marg_data.H = Hm;
  marg_data.order = order;
}

template <class Scalar>
void SqrtKeypointVioEstimator<Scalar>::removeZeroKeyframeFromMargData(FrameId torm_ts) {
  size_t total_rows = marg_data.H.rows();
  size_t old_total_cols = marg_data.H.cols();
  size_t new_total_cols = old_total_cols - POSE_SIZE;
  MatX Hm;
  Hm.setZero(total_rows, new_total_cols);

  AbsOrderMap order{};
  size_t idx = SIZE_MAX;
  for (const auto& [ts, idxsize] : marg_data.order.abs_order_map) {
    if (torm_ts == ts) {
      idx = order.total_size;
      continue;
    }

    const auto& [_, size] = idxsize;
    order.abs_order_map[ts] = std::make_pair(order.total_size, size);
    order.total_size += size;
    order.items++;
  }

  if (idx == SIZE_MAX) return; // torm_ts not in the marg_data absolute order map

  size_t bef = idx;                       // Side size to the left (before)
  size_t aft = new_total_cols - idx ;     // Cols after idx (after)
  size_t i = idx;                         // Index to remove at
  size_t s = POSE_SIZE;                   // Number of inserted vars

  if (i == 0) {                                                                                         // If at the beginning
    Hm.template block(0, 0, total_rows, aft) = marg_data.H.template block(0, i + s, total_rows, aft);   // right
  } else if (i == order.total_size) {                                                                   // If at the end
    Hm.template block(0, 0, total_rows, bef) = marg_data.H.template block(0, 0, total_rows, bef);       // left
  } else {                                                                                              // If in the middle
    Hm.template block(0, 0, total_rows, bef) = marg_data.H.template block(0, 0, total_rows, bef);       // left
    Hm.template block(0, i, total_rows, aft) = marg_data.H.template block(0, i + s, total_rows, aft);   // right
  }

  marg_data.H = Hm;
  marg_data.order = order;
}

template <class Scalar>
void SqrtKeypointVioEstimator<Scalar>::addCovisibilityMap() {
  if (show_uimat(UIMAT::HB_M_PRIOR_BEFORE)) {
    visual_data->geth(UIMAT::HB_M_PRIOR_BEFORE).H =
        std::make_shared<Eigen::MatrixXf>(marg_data.H.template cast<float>());
    visual_data->geth(UIMAT::HB_M_PRIOR_BEFORE).b =
        std::make_shared<Eigen::VectorXf>(marg_data.b.template cast<float>());
    visual_data->geth(UIMAT::HB_M_PRIOR_BEFORE).aom = std::make_shared<AbsOrderMap>(marg_data.order);
  }

  for (auto& [kf_id, pos] : covisible_submap->getKeyframes()) {
    if (kf_ids.find(kf_id) == kf_ids.end() && ltkfs.find(kf_id) == ltkfs.end()) {
      // Add Prior Keyframe as long term keyframe
      ltkfs.emplace(kf_id);
      tmpkfs.emplace(kf_id);
      // Add pose to poses state window
      frame_poses.emplace(kf_id, PoseStateWithLin(kf_id, pos.template cast<Scalar>(), true));
      if (out_vis_queue) visual_data->ltframes[kf_id] = pos.template cast<double>();
      // Adjust Marg Data to the new window of keyframes
      addZeroKeyframeToMargData(kf_id);
    }
  }

  if (show_uimat(UIMAT::HB_M_PRIOR_AFTER)) {
    visual_data->geth(UIMAT::HB_M_PRIOR_AFTER).H =
        std::make_shared<Eigen::MatrixXf>(marg_data.H.template cast<float>());
    visual_data->geth(UIMAT::HB_M_PRIOR_AFTER).b =
        std::make_shared<Eigen::VectorXf>(marg_data.b.template cast<float>());
    visual_data->geth(UIMAT::HB_M_PRIOR_AFTER).aom = std::make_shared<AbsOrderMap>(marg_data.order);
  }

  lmdb.mergeLMDB(covisible_submap, false);
}

template <class Scalar>
void SqrtKeypointVioEstimator<Scalar>::removeCovisibilityMap() {
  std::set<FrameId> keyframes_to_rm{};
  std::set<FrameId> poses_to_marg{};
  std::set<FrameId> states_to_marg_all{};

  if (show_uimat(UIMAT::HB_M_PRIOR_BEFORE)) {
    visual_data->geth(UIMAT::HB_M_PRIOR_BEFORE).H =
        std::make_shared<Eigen::MatrixXf>(marg_data.H.template cast<float>());
    visual_data->geth(UIMAT::HB_M_PRIOR_BEFORE).b =
        std::make_shared<Eigen::VectorXf>(marg_data.b.template cast<float>());
    visual_data->geth(UIMAT::HB_M_PRIOR_BEFORE).aom = std::make_shared<AbsOrderMap>(marg_data.order);
  }

  for (const auto& kf_id : tmpkfs) {
    if (ltkfs.count(kf_id) > 0) {
      ltkfs.erase(kf_id);
      frame_poses.erase(kf_id);
      keyframes_to_rm.emplace(kf_id);
      if (out_vis_queue) visual_data->ltframes.erase(kf_id);
      removeZeroKeyframeFromMargData(kf_id);
    }
  }
  tmpkfs.clear();

  if (show_uimat(UIMAT::HB_M_PRIOR_AFTER)) {
    visual_data->geth(UIMAT::HB_M_PRIOR_AFTER).H =
        std::make_shared<Eigen::MatrixXf>(marg_data.H.template cast<float>());
    visual_data->geth(UIMAT::HB_M_PRIOR_AFTER).b =
        std::make_shared<Eigen::VectorXf>(marg_data.b.template cast<float>());
    visual_data->geth(UIMAT::HB_M_PRIOR_AFTER).aom = std::make_shared<AbsOrderMap>(marg_data.order);
  }

  lmdb.removeKeyframes(keyframes_to_rm, poses_to_marg, states_to_marg_all);
}

template <class Scalar>
bool SqrtKeypointVioEstimator<Scalar>::resetState(typename IntegratedImuMeasurement<Scalar>::Ptr& meas,
                                                  OpticalFlowResult::Ptr& curr_frame,
                                                  OpticalFlowResult::Ptr& prev_frame) {
  meas = nullptr;
  curr_frame = nullptr;
  prev_frame = nullptr;

  typename ImuData<Scalar>::Ptr data = popFromImuDataQueue();
  if (data == nullptr) return true;
  data->accel = calib.calib_accel_bias.getCalibrated(data->accel);
  data->gyro = calib.calib_gyro_bias.getCalibrated(data->gyro);

  initialized = false;
  last_processed_t_ns = 0;
  // drain_input_queues();
  frame_states.clear();
  frame_poses.clear();
  frame_idx.clear();
  lmdb.clear();

  take_kf = true;
  frames_after_kf = 0;
  frame_count = 0;
  kf_ids.clear();
  ltkfs.clear();
  imu_meas.clear();
  prev_opt_flow_res.clear();
  num_points_kf.clear();

  // Setup marginalization
  marg_data.is_sqrt = config.vio_sqrt_marg;
  marg_data.order.abs_order_map.clear();
  marg_data.order.items = 0;
  marg_data.order.total_size = 0;
  marg_data.H.setZero(POSE_VEL_BIAS_SIZE, POSE_VEL_BIAS_SIZE);
  marg_data.b.setZero(POSE_VEL_BIAS_SIZE);

  // Version without prior
  nullspace_marg_data.is_sqrt = marg_data.is_sqrt;
  nullspace_marg_data.order.abs_order_map.clear();
  nullspace_marg_data.order.items = 0;
  nullspace_marg_data.order.total_size = 0;
  nullspace_marg_data.H.setZero(POSE_VEL_BIAS_SIZE, POSE_VEL_BIAS_SIZE);
  nullspace_marg_data.b.setZero(POSE_VEL_BIAS_SIZE);

  if (marg_data.is_sqrt) {
    // prior on position
    marg_data.H.diagonal().template head<3>().setConstant(std::sqrt(Scalar(config.vio_init_pose_weight)));
    // prior on yaw
    marg_data.H(5, 5) = std::sqrt(Scalar(config.vio_init_pose_weight));

    // small prior to avoid jumps in bias
    marg_data.H.diagonal().template segment<3>(9).array() = std::sqrt(Scalar(config.vio_init_ba_weight));
    marg_data.H.diagonal().template segment<3>(12).array() = std::sqrt(Scalar(config.vio_init_bg_weight));
  } else {
    // prior on position
    marg_data.H.diagonal().template head<3>().setConstant(Scalar(config.vio_init_pose_weight));
    // prior on yaw
    marg_data.H(5, 5) = Scalar(config.vio_init_pose_weight);

    // small prior to avoid jumps in bias
    marg_data.H.diagonal().template segment<3>(9).array() = Scalar(config.vio_init_ba_weight);
    marg_data.H.diagonal().template segment<3>(12).array() = Scalar(config.vio_init_bg_weight);
  }

  std::cout << "marg_H (sqrt:" << marg_data.is_sqrt << ")\n" << marg_data.H << std::endl;

  opt_started = false;
  schedule_reset = false;
  return false;
}

template <class Scalar_>
void SqrtKeypointVioEstimator<Scalar_>::initialize(int64_t t_ns, const Sophus::SE3d& T_w_i,
                                                   const Eigen::Vector3d& vel_w_i, const Eigen::Vector3d& bg,
                                                   const Eigen::Vector3d& ba) {
  initialized = true;
  T_w_i_init = T_w_i.cast<Scalar>();

  last_state_t_ns = t_ns;
  imu_meas[t_ns] = IntegratedImuMeasurement<Scalar>(t_ns, bg.cast<Scalar>(), ba.cast<Scalar>());
  frame_states[t_ns] = PoseVelBiasStateWithLin<Scalar>(t_ns, T_w_i_init, vel_w_i.cast<Scalar>(), bg.cast<Scalar>(),
                                                       ba.cast<Scalar>(), true);
  frame_idx[t_ns] = frame_count++;

  marg_data.order.abs_order_map[t_ns] = std::make_pair(0, POSE_VEL_BIAS_SIZE);
  marg_data.order.total_size = POSE_VEL_BIAS_SIZE;
  marg_data.order.items = 1;

  nullspace_marg_data.order = marg_data.order;

  initialize(bg, ba);
}

template <class Scalar_>
void SqrtKeypointVioEstimator<Scalar_>::initialize(const Eigen::Vector3d& bg_, const Eigen::Vector3d& ba_) {
  Vec3 bg_init = bg_.cast<Scalar>();
  Vec3 ba_init = ba_.cast<Scalar>();

  auto proc_func = [&, bg = bg_init, ba = ba_init] {
    OpticalFlowResult::Ptr prev_frame, curr_frame;
    typename IntegratedImuMeasurement<Scalar>::Ptr meas;

    const Vec3 accel_cov = calib.dicrete_time_accel_noise_std().array().square();
    const Vec3 gyro_cov = calib.dicrete_time_gyro_noise_std().array().square();

    typename ImuData<Scalar>::Ptr data = popFromImuDataQueue();

    bool run = data != nullptr;  // End VIO otherwise
    if (run) {
      data->accel = calib.calib_accel_bias.getCalibrated(data->accel);
      data->gyro = calib.calib_gyro_bias.getCalibrated(data->gyro);
    }

    while (run) {
      bool reset_performed = schedule_reset;
      if (reset_performed) {
        bool exit_requested = resetState(meas, curr_frame, prev_frame);
        if (exit_requested) break;
      }

      vision_data_queue.pop(curr_frame);

      if (config.vio_enforce_realtime) {
        // drop current frame if another frame is already in the queue.
        while (!vision_data_queue.empty()) vision_data_queue.pop(curr_frame);
      }

      if (!curr_frame.get()) {
        break;
      }
      curr_frame->input_images->addTime("vio_start");
      curr_frame->input_images->state_reset = reset_performed;

      // Correct camera time offset
      // curr_frame->t_ns += calib.cam_time_offset_ns;

      if (out_vis_queue) {
        visual_data = std::make_shared<VioVisualizationData>();
        visual_data->t_ns = curr_frame->t_ns;
      }
      if (out_vio_data_queue) {
        map_stamp = std::make_shared<MapStamp>();
        map_stamp->t_ns = curr_frame->t_ns;
      }

      if (!initialized) {
        while (data->t_ns < curr_frame->t_ns) {
          data = popFromImuDataQueue();
          if (!data) break;
          data->accel = calib.calib_accel_bias.getCalibrated(data->accel);
          data->gyro = calib.calib_gyro_bias.getCalibrated(data->gyro);
          // std::cout << "Skipping IMU data.." << std::endl;
        }

        Vec3 vel_w_i_init;
        vel_w_i_init.setZero();

        T_w_i_init.translation().setZero();
        T_w_i_init.setQuaternion(Eigen::Quaternion<Scalar>::FromTwoVectors(data->accel, Vec3::UnitZ()));

        last_state_t_ns = curr_frame->t_ns;
        imu_meas[last_state_t_ns] = IntegratedImuMeasurement<Scalar>(last_state_t_ns, bg, ba);
        frame_states[last_state_t_ns] =
            PoseVelBiasStateWithLin<Scalar>(last_state_t_ns, T_w_i_init, vel_w_i_init, bg, ba, true);
        frame_idx[last_state_t_ns] = frame_count++;

        marg_data.order.abs_order_map[last_state_t_ns] = std::make_pair(0, POSE_VEL_BIAS_SIZE);
        marg_data.order.total_size = POSE_VEL_BIAS_SIZE;
        marg_data.order.items = 1;

        std::cout << "Setting up filter: t_ns " << last_state_t_ns << std::endl;
        std::cout << "T_w_i\n" << T_w_i_init.matrix() << std::endl;
        std::cout << "vel_w_i " << vel_w_i_init.transpose() << std::endl;

        if (config.vio_debug || config.vio_extended_logging) {
          logMargNullspace();
        }

        initialized = true;
      }

      if (prev_frame) {
        // preintegrate measurements

        auto last_state = frame_states.at(last_state_t_ns);

        meas.reset(new IntegratedImuMeasurement<Scalar>(prev_frame->t_ns, last_state.getState().bias_gyro,
                                                        last_state.getState().bias_accel));

        BASALT_ASSERT_MSG(prev_frame->t_ns != curr_frame->t_ns,
                          "duplicate frame timestamps?! zero time delta leads "
                          "to invalid IMU integration.");

        BASALT_ASSERT_MSG(prev_frame->t_ns < curr_frame->t_ns,
                          "frame timestamps not monotonically increasing?! "
                          "are we going 88mph and inside a delorean?");

        while (data->t_ns <= prev_frame->t_ns) {
          data = popFromImuDataQueue();
          if (!data) break;
          data->accel = calib.calib_accel_bias.getCalibrated(data->accel);
          data->gyro = calib.calib_gyro_bias.getCalibrated(data->gyro);
        }

        while (data->t_ns <= curr_frame->t_ns) {
          meas->integrate(*data, accel_cov, gyro_cov);
          data = popFromImuDataQueue();
          if (!data) break;
          data->accel = calib.calib_accel_bias.getCalibrated(data->accel);
          data->gyro = calib.calib_gyro_bias.getCalibrated(data->gyro);
        }

        if (meas->get_start_t_ns() + meas->get_dt_ns() < curr_frame->t_ns) {
          if (!data.get()) break;
          int64_t tmp = data->t_ns;
          data->t_ns = curr_frame->t_ns;
          meas->integrate(*data, accel_cov, gyro_cov);
          data->t_ns = tmp;
        }
      }
      curr_frame->input_images->addTime("imu_preintegrated");

      bool send_covisibility_request = out_covi_req_queue && ((config.vio_covisibility_query_frequency != 0 && frame_count % config.vio_covisibility_query_frequency == 0) || get_map);
      if (send_covisibility_request) {
        if (covisible_submap != nullptr) removeCovisibilityMap();
        std::vector<KeypointId> keypoint_ids;
        for (size_t i = 0; i < curr_frame->keypoints.size(); i++) {
          for (const auto& kv : curr_frame->keypoints[i]) keypoint_ids.emplace_back(kv.first);
        }
        // TODO: doing the request here leaves a frame without any covisibility map
        out_covi_req_queue->push(std::make_shared<std::vector<KeypointId>>(keypoint_ids));
        get_map = false;
      }

      while (in_covi_res_queue.try_pop(covisible_submap)) addCovisibilityMap();

      bool success = measure(curr_frame, meas);
      if (!success) {
        schedule_reset = true;
        continue;
      }

      prev_frame = curr_frame;
    }

    if (out_vis_queue) out_vis_queue->push(nullptr);
    if (out_marg_queue) out_marg_queue->push(nullptr);
    if (out_vio_data_queue) out_vio_data_queue->push(nullptr);
    if (out_state_queue) out_state_queue->push(nullptr);
    if (out_covi_req_queue) out_covi_req_queue->push(nullptr);

    finished = true;

    std::cout << "Finished VIOFilter " << std::endl;
  };

  processing_thread.reset(new std::thread(proc_func));
}

template <class Scalar_>
void SqrtKeypointVioEstimator<Scalar_>::addIMUToQueue(const ImuData<double>::Ptr& data) {
  imu_data_queue.emplace(data);
}

template <class Scalar_>
void SqrtKeypointVioEstimator<Scalar_>::addVisionToQueue(const OpticalFlowResult::Ptr& data) {
  vision_data_queue.push(data);
}

template <class Scalar_>
typename ImuData<Scalar_>::Ptr SqrtKeypointVioEstimator<Scalar_>::popFromImuDataQueue() {
  ImuData<double>::Ptr data;
  imu_data_queue.pop(data);

  if constexpr (std::is_same_v<Scalar, double>) {
    return data;
  } else {
    typename ImuData<Scalar>::Ptr data2;
    if (data) {
      data2.reset(new ImuData<Scalar>);
      *data2 = data->cast<Scalar>();
    }
    return data2;
  }
}

template <class Scalar_>
bool SqrtKeypointVioEstimator<Scalar_>::measure(const OpticalFlowResult::Ptr& opt_flow_meas,
                                                const typename IntegratedImuMeasurement<Scalar>::Ptr& meas) {
  stats_sums_.add("frame_id", opt_flow_meas->t_ns).format("none");
  Timer t_total;

  if (meas.get()) {
    BASALT_ASSERT(frame_states[last_state_t_ns].getState().t_ns == meas->get_start_t_ns());
    BASALT_ASSERT(opt_flow_meas->t_ns == meas->get_dt_ns() + meas->get_start_t_ns());
    BASALT_ASSERT(meas->get_dt_ns() > 0);

    PoseVelBiasState<Scalar> next_state = frame_states.at(last_state_t_ns).getState();

    meas->predictState(frame_states.at(last_state_t_ns).getState(), g, next_state);

    last_state_t_ns = opt_flow_meas->t_ns;
    next_state.t_ns = opt_flow_meas->t_ns;

    frame_states[last_state_t_ns] = PoseVelBiasStateWithLin<Scalar>(next_state);
    frame_idx[last_state_t_ns] = frame_count++;

    imu_meas[meas->get_start_t_ns()] = *meas;
  }

  // save results
  prev_opt_flow_res[opt_flow_meas->t_ns] = opt_flow_meas;

  // Make new residual for existing keypoints
  int NUM_CAMS = opt_flow_meas->keypoints.size();
  std::vector<int> connected(NUM_CAMS, 0);
  std::map<int64_t, int> num_points_connected;
  std::vector<std::unordered_set<int>> unconnected_obs(NUM_CAMS);
  for (int i = 0; i < NUM_CAMS; i++) {
    TimeCamId tcid_target(opt_flow_meas->t_ns, i);

    for (const auto& kv_obs : opt_flow_meas->keypoints[i]) {
      int kpt_id = kv_obs.first;

      if (lmdb.landmarkExists(kpt_id)) {
        const TimeCamId& tcid_host = lmdb.getLandmark(kpt_id).host_kf_id;

        KeypointObservation<Scalar> kobs;
        kobs.kpt_id = kpt_id;
        kobs.pos = kv_obs.second.translation().cast<Scalar>();

        lmdb.addObservation(tcid_target, kobs);
        // obs[tcid_host][tcid_target].push_back(kobs);

        if (num_points_connected.count(tcid_host.frame_id) == 0) {
          num_points_connected[tcid_host.frame_id] = 0;
        }
        num_points_connected[tcid_host.frame_id]++;

        connected[i]++;
      } else {
        unconnected_obs[i].emplace(kpt_id);
      }
    }
  }

  if (Scalar(connected[0]) / (connected[0] + unconnected_obs[0].size()) < Scalar(config.vio_new_kf_keypoints_thresh) &&
      frames_after_kf > config.vio_min_frames_after_kf)
    take_kf = true;

  if (config.vio_debug) {
    for (int i = 0; i < NUM_CAMS; i++) {
      std::cout << "connected[" << i << "] = " << connected[i] << ", "
                << "unconnected[" << i << "] =" << unconnected_obs[i].size() << std::endl;
    }
  }

  if (take_ltkf) {
    if (!kf_ids.empty()) {  // Move newest kf to ltkfs
      auto last_kf_it = std::prev(kf_ids.end());
      ltkfs.emplace(*last_kf_it);
      kf_ids.erase(last_kf_it);
    }
    take_ltkf = false;
  }

  if (take_kf) {
    // Triangulate new points from one of the observations (with sufficient
    // baseline) and make keyframe
    take_kf = false;
    frames_after_kf = 0;
    kf_ids.emplace(last_state_t_ns);
    if (visual_data) visual_data->keyframed_idx[last_state_t_ns] = frame_idx.at(last_state_t_ns);
    const SE3 pos = getPoseStateWithLin(last_state_t_ns).getPose();
    lmdb.addKeyframe(last_state_t_ns, frame_idx.at(last_state_t_ns), pos);

    int num_points_added = 0;
    for (int i = 0; i < NUM_CAMS; i++) {
      TimeCamId tcidl(opt_flow_meas->t_ns, i);

      for (int lm_id : unconnected_obs[i]) {
        if (lmdb.landmarkExists(lm_id)) continue;
        // Find all observations
        std::map<TimeCamId, KeypointObservation<Scalar>> kp_obs;

        for (const auto& kv : prev_opt_flow_res) {
          for (size_t k = 0; k < kv.second->keypoints.size(); k++) {
            auto it = kv.second->keypoints[k].find(lm_id);
            if (it != kv.second->keypoints[k].end()) {
              TimeCamId tcido(kv.first, k);

              KeypointObservation<Scalar> kobs;
              kobs.kpt_id = lm_id;
              kobs.pos = it->second.translation().template cast<Scalar>();

              // obs[tcidl][tcido].push_back(kobs);
              kp_obs[tcido] = kobs;
            }
          }
        }

        // triangulate
        bool valid_kp = false;
        const Scalar min_triang_distance2 =
            Scalar(config.vio_min_triangulation_dist * config.vio_min_triangulation_dist);
        for (const auto& kv_obs : kp_obs) {
          if (valid_kp) break;
          TimeCamId tcido = kv_obs.first;

          const Vec2 p0 = opt_flow_meas->keypoints.at(i).at(lm_id).translation().cast<Scalar>();
          const Vec2 p1 = prev_opt_flow_res[tcido.frame_id]
                              ->keypoints[tcido.cam_id]
                              .at(lm_id)
                              .translation()
                              .template cast<Scalar>();

          Vec4 p0_3d, p1_3d;
          bool valid1 = calib.intrinsics[i].unproject(p0, p0_3d);
          bool valid2 = calib.intrinsics[tcido.cam_id].unproject(p1, p1_3d);
          if (!valid1 || !valid2) continue;

          SE3 T_i0_i1 =
              getPoseStateWithLin(tcidl.frame_id).getPose().inverse() * getPoseStateWithLin(tcido.frame_id).getPose();
          SE3 T_0_1 = calib.T_i_c[i].inverse() * T_i0_i1 * calib.T_i_c[tcido.cam_id];

          if (T_0_1.translation().squaredNorm() < min_triang_distance2) continue;

          Vec4 p0_triangulated = triangulate(p0_3d.template head<3>(), p1_3d.template head<3>(), T_0_1);

          if (p0_triangulated.array().isFinite().all() && p0_triangulated[3] > 0 && p0_triangulated[3] < 3.0) {
            Landmark<Scalar> lm_pos;
            lm_pos.host_kf_id = tcidl;
            lm_pos.direction = StereographicParam<Scalar>::project(p0_triangulated);
            lm_pos.inv_dist = p0_triangulated[3];
            lmdb.addLandmark(lm_id, lm_pos);

            num_points_added++;
            valid_kp = true;
          }
        }

        if (valid_kp) {
          for (const auto& kv_obs : kp_obs) {
            lmdb.addObservation(kv_obs.first, kv_obs.second);
          }
        }
      }
    }

    num_points_kf[opt_flow_meas->t_ns] = num_points_added;
  } else {
    frames_after_kf++;
  }

  std::unordered_set<KeypointId> lost_landmaks;
  if (config.vio_marg_lost_landmarks) {
    for (const auto& kv : lmdb.getLandmarks()) {
      bool connected = false;
      for (size_t i = 0; i < opt_flow_meas->keypoints.size(); i++) {
        if (opt_flow_meas->keypoints[i].count(kv.first) > 0) connected = true;
      }
      if (!connected) {
        lost_landmaks.emplace(kv.first);
      }
    }
  }
  opt_flow_meas->input_images->addTime("landmarks_updated");

  bool success = optimize_and_marg(opt_flow_meas->input_images, num_points_connected, lost_landmaks);
  if (!success) return false;

  size_t num_cams = opt_flow_meas->keypoints.size();
  bool features_cap = (opt_flow_meas->input_images->stats.enabled_caps & VIT_TRACKER_POSE_CAPABILITY_FEATURES) != 0;
  bool avg_depth_needed =
      opt_flow_depth_guess_queue && config.optical_flow_matching_guess_type == MatchingGuessType::REPROJ_AVG_DEPTH;

  using Projections = std::vector<Eigen::aligned_vector<Eigen::Vector4d>>;
  std::shared_ptr<Projections> projections = nullptr;
  if (features_cap || out_vis_queue || avg_depth_needed) {
    projections = std::make_shared<Projections>(num_cams);
    computeProjections(*projections, last_state_t_ns);
  }

  if (out_state_queue) {
    PoseVelBiasStateWithLin p = frame_states.at(last_state_t_ns);

    typename PoseVelBiasState<double>::Ptr data(new PoseVelBiasState<double>(p.getState().template cast<double>()));

    data->input_images = opt_flow_meas->input_images;
    data->input_images->addTime("pose_produced");

    if (avg_depth_needed) {
      double avg_invdepth = 0;
      double num_features = 0;
      for (const auto& cam_projs : *projections) {
        for (const Eigen::Vector4d& v : cam_projs) avg_invdepth += v.z();
        num_features += cam_projs.size();
      }

      bool valid = avg_invdepth > 0 && num_features > 0;
      float default_depth = config.optical_flow_matching_default_depth;
      double avg_depth = valid ? num_features / avg_invdepth : default_depth;

      if (opt_flow_depth_guess_queue) opt_flow_depth_guess_queue->push(avg_depth);
    }

    if (features_cap) {
      for (size_t i = 0; i < num_cams; i++) {
        for (const Eigen::Vector4d& v : projections->at(i)) {
          vit::PoseFeature lm = {};
          lm.id = v.w();
          lm.u = v.x();
          lm.v = v.y();
          lm.depth = v.z();
          data->input_images->stats.addFeature(i, lm);
        }
      }
    }
    if (out_state_queue) out_state_queue->push(data);
    if (opt_flow_state_queue) opt_flow_state_queue->push(data);
  }

  bool lm_bundle_needed = opt_flow_lm_bundle_queue && config.optical_flow_recall_enable;
  if (lm_bundle_needed) {
    LandmarkBundle::Ptr lmb = std::make_shared<LandmarkBundle>();
    lmb->ts = last_state_t_ns;
    // TODO@brunozanotti: improve landmark ordering: this is to prioritize older landmarks during recall
    std::map<LandmarkId, Landmark<Scalar_>> ordered_lms{};
    auto& unordered_lms = lmdb.getLandmarks();
    ordered_lms.insert(unordered_lms.begin(), unordered_lms.end());
    for (const auto& [lmid, lm] : ordered_lms) {
      if (frame_poses.count(lm.host_kf_id.frame_id) == 0) continue;
      SE3 T_w_i = frame_poses.at(lm.host_kf_id.frame_id).getPose();
      SE3 T_i_c = calib.T_i_c[lm.host_kf_id.cam_id];
      SE3 T_w_c = T_w_i * T_i_c;

      Vec4 pt_c = StereographicParam<Scalar>::unproject(lm.direction);
      pt_c *= 1 / lm.inv_dist;  // scale by depth
      pt_c[3] = 1;

      Vec4 pt_w = T_w_c * pt_c;
      lmb->lmids.emplace_back(lmid);
      lmb->lms.emplace_back(pt_w.template cast<float>());
    }
    opt_flow_lm_bundle_queue->push(lmb);
  }

  if (out_vis_queue) {
    for (const auto& [ts, p] : frame_states) visual_data->states[ts] = p.getState().T_w_i.template cast<double>();

    for (const auto& [ts, pstate] : frame_poses) {
      auto& frames = kf_ids.count(ts) ? visual_data->frames : visual_data->ltframes;
      frames[ts] = pstate.getPose().template cast<double>();
    }

    visual_data->frame_idx = frame_idx;

    get_current_points(visual_data->points, visual_data->point_ids);

    visual_data->projections = projections;

    visual_data->opt_flow_res = prev_opt_flow_res[last_state_t_ns];

    out_vis_queue->push(visual_data);
  }

  last_processed_t_ns = last_state_t_ns;

  stats_sums_.add("measure", t_total.elapsed()).format("ms");

  return true;
}

template <class Scalar_>
void SqrtKeypointVioEstimator<Scalar_>::logMargNullspace() {
  nullspace_marg_data.order = marg_data.order;
  if (config.vio_debug) {
    std::cout << "======== Marg nullspace ==========" << std::endl;
    stats_sums_.add("marg_ns", checkMargNullspace());
    std::cout << "=================================" << std::endl;
  } else {
    stats_sums_.add("marg_ns", checkMargNullspace());
  }
  stats_sums_.add("marg_ev", checkMargEigenvalues());
}

template <class Scalar_>
Eigen::VectorXd SqrtKeypointVioEstimator<Scalar_>::checkMargNullspace() const {
  return checkNullspace(nullspace_marg_data, frame_states, frame_poses, config.vio_debug);
}

template <class Scalar_>
Eigen::VectorXd SqrtKeypointVioEstimator<Scalar_>::checkMargEigenvalues() const {
  return checkEigenvalues(nullspace_marg_data, false);
}

template <class Scalar>
bool SqrtKeypointVioEstimator<Scalar>::show_uimat(UIMAT m) const {
  UIMAT showed = prev_opt_flow_res.at(last_state_t_ns)->input_images->show_uimat;

  bool show_none = showed == UIMAT::NONE;
  if (show_none) return false;

  bool ui_enabled = out_vis_queue != nullptr;
  bool show_all = showed == UIMAT::ALL;
  bool show_this = showed == m;
  bool res = ui_enabled && (show_all || show_this);
  return res;
}

template <class Scalar_>
bool SqrtKeypointVioEstimator<Scalar_>::marginalize(const std::map<int64_t, int>& num_points_connected,
                                                    const std::unordered_set<KeypointId>& lost_landmaks) {
  if (!opt_started) return true;

  Timer t_total;

  if (frame_poses.size() > ltkfs.size() + max_kfs || frame_states.size() >= max_states) {
    // Marginalize

    const int states_to_remove = frame_states.size() - max_states + 1;

    auto it = frame_states.cbegin();
    for (int i = 0; i < states_to_remove; i++) it++;
    int64_t last_state_to_marg = it->first;

    AbsOrderMap aom;

    // remove all frame_poses that are not kfs
    std::set<int64_t> poses_to_marg;
    for (const auto& kv : frame_poses) {
      aom.abs_order_map[kv.first] = std::make_pair(aom.total_size, POSE_SIZE);

      if (kf_ids.count(kv.first) == 0 && ltkfs.count(kv.first) == 0) poses_to_marg.emplace(kv.first);

      // Check that we have the same order as marginalization
      BASALT_ASSERT(marg_data.order.abs_order_map.at(kv.first) == aom.abs_order_map.at(kv.first));

      aom.total_size += POSE_SIZE;
      aom.items++;
    }

    std::set<int64_t> states_to_marg_vel_bias;
    std::set<int64_t> states_to_marg_all;
    for (const auto& kv : frame_states) {
      if (kv.first > last_state_to_marg) break;

      if (kv.first != last_state_to_marg) {
        if (kf_ids.count(kv.first) > 0 || ltkfs.count(kv.first) > 0) {
          states_to_marg_vel_bias.emplace(kv.first);
        } else {
          states_to_marg_all.emplace(kv.first);
        }
      }

      aom.abs_order_map[kv.first] = std::make_pair(aom.total_size, POSE_VEL_BIAS_SIZE);

      // Check that we have the same order as marginalization
      if (aom.items < marg_data.order.abs_order_map.size())
        BASALT_ASSERT(marg_data.order.abs_order_map.at(kv.first) == aom.abs_order_map.at(kv.first));

      aom.total_size += POSE_VEL_BIAS_SIZE;
      aom.items++;
    }

    auto kf_ids_all = kf_ids;
    std::set<int64_t> kfs_to_marg;
    while (kf_ids.size() > max_kfs && !states_to_marg_vel_bias.empty()) {
      int64_t id_to_marg = -1;

      if (config.vio_kf_marg_criteria == KeyframeMargCriteria::KF_MARG_FORWARD_VECTOR) {
        // TODO: With feature recall enabled, we needed a better marginalization
        // criteria since "unconnected observations" is now not always something
        // bad because they can be reconnected. The KF_MARG_FORWARD_VECTOR
        // criteria tries to keep keyframes with forward vectors as spread as
        // possible but it is just a basic approach. A more complete approach
        // should also prioritize keyframes that: are older, have more features,
        // have better quality features.
        if (kf_ids.size() > 2 && id_to_marg < 0) {
          std::set<int64_t> all_kfs = ltkfs;
          all_kfs.insert(kf_ids.begin(), kf_ids.end());

          // Do not account for the two most recent frames
          auto last1 = std::prev(kf_ids.end(), 2);
          auto last2 = std::prev(all_kfs.end(), 2);
          Scalar min_score = std::numeric_limits<Scalar>::max();
          int64_t min_score_id = -1;

          auto get_forward_vector2d = [&](int64_t ts) -> Vec2 {
            SE3 T_w_i = frame_poses.at(ts).getPose();
            SE3 T_w_c0 = T_w_i * calib.T_i_c[0];
            Vec3 fwd3d = T_w_c0.so3() * Vec3{0, 0, 1};
            Vec2 fwd2d = fwd3d.template head<2>();
            return fwd2d;
          };

          for (auto it1 = kf_ids.begin(); it1 != last1; ++it1) {
            Vec2 fwd1 = get_forward_vector2d(*it1);
            Scalar score = 0;
            for (auto it2 = all_kfs.begin(); it2 != last2; ++it2) {
              Vec2 fwd2 = get_forward_vector2d(*it2);
              Scalar dot = std::clamp(fwd1.dot(fwd2), Scalar(-1), Scalar(1));  // clamp needed, otherwise acos can fail
              Scalar angle = acos(dot);
              score += angle;
            }

            if (score < min_score) {
              min_score_id = *it1;
              min_score = score;
            }
          }

          id_to_marg = min_score_id;
        }
      } else if (config.vio_kf_marg_criteria == KeyframeMargCriteria::KF_MARG_DEFAULT) {
        // starting from the oldest kf (and skipping the newest 2 kfs), try to
        // find a kf that has less than a small percentage of it's landmarks
        // tracked by the current frame
        if (kf_ids.size() > 2) {
          // Note: size > 2 check is to ensure prev(kf_ids.end(), 2) is valid
          auto end_minus_2 = std::prev(kf_ids.end(), 2);

          for (auto it = kf_ids.begin(); it != end_minus_2; ++it) {
            if (num_points_connected.count(*it) == 0 ||
                (num_points_connected.at(*it) / static_cast<float>(num_points_kf.at(*it)) <
                 config.vio_kf_marg_feature_ratio)) {
              id_to_marg = *it;
              break;
            }
          }
        }

        // Note: This score function is taken from DSO, but it seems to mostly
        // marginalize the oldest keyframe. This may be due to the fact that
        // we don't have as long-lived landmarks, which may change if we ever
        // implement "rediscovering" of lost feature tracks by projecting
        // untracked landmarks into the localized frame.
        if (kf_ids.size() > 2 && id_to_marg < 0) {
          // Note: size > 2 check is to ensure prev(kf_ids.end(), 2) is valid
          auto end_minus_2 = std::prev(kf_ids.end(), 2);

          int64_t last_kf = *kf_ids.crbegin();
          Scalar min_score = std::numeric_limits<Scalar>::max();
          int64_t min_score_id = -1;

          for (auto it1 = kf_ids.begin(); it1 != end_minus_2; ++it1) {
            // small distance to other keyframes --> higher score
            Scalar denom = 0;
            for (auto it2 = kf_ids.begin(); it2 != end_minus_2; ++it2) {
              denom +=
                  1 / ((frame_poses.at(*it1).getPose().translation() - frame_poses.at(*it2).getPose().translation())
                           .norm() +
                       Scalar(1e-5));
            }

            // small distance to latest kf --> lower score
            Scalar score = std::sqrt((frame_poses.at(*it1).getPose().translation() -
                                      frame_states.at(last_kf).getState().T_w_i.translation())
                                         .norm()) *
                           denom;

            if (score < min_score) {
              min_score_id = *it1;
              min_score = score;
            }
          }

          id_to_marg = min_score_id;
        }
      } else {
        BASALT_LOG_FATAL("Unexpected marginalization criteria");
      }

      // if no frame was selected, the logic above is faulty
      BASALT_ASSERT(id_to_marg >= 0);

      kfs_to_marg.emplace(id_to_marg);
      poses_to_marg.emplace(id_to_marg);

      kf_ids.erase(id_to_marg);
    }

    //    std::cout << "marg order" << std::endl;
    //    aom.print_order();

    //    std::cout << "marg prior order" << std::endl;
    //    marg_order.print_order();

    if (config.vio_debug) {
      std::cout << "states_to_remove " << states_to_remove << std::endl;
      std::cout << "poses_to_marg.size() " << poses_to_marg.size() << std::endl;
      std::cout << "states_to_marg.size() " << states_to_marg_all.size() << std::endl;
      std::cout << "state_to_marg_vel_bias.size() " << states_to_marg_vel_bias.size() << std::endl;
      std::cout << "kfs_to_marg.size() " << kfs_to_marg.size() << std::endl;
    }

    Timer t_actual_marg;

    size_t asize = aom.total_size;

    bool is_lin_sqrt = isLinearizationSqrt(config.vio_linearization_type);

    MatX Q2Jp_or_H;
    VecX Q2r_or_b;

    {
      Timer t_linearize;

      typename LinearizationBase<Scalar, POSE_SIZE>::Options lqr_options;
      lqr_options.lb_options.huber_parameter = huber_thresh;
      lqr_options.lb_options.obs_std_dev = obs_std_dev;
      lqr_options.linearization_type = config.vio_linearization_type;

      ImuLinData<Scalar> ild = {g, gyro_bias_sqrt_weight, accel_bias_sqrt_weight, {}};

      for (const auto& kv : imu_meas) {
        int64_t start_t = kv.second.get_start_t_ns();
        int64_t end_t = kv.second.get_start_t_ns() + kv.second.get_dt_ns();

        if (aom.abs_order_map.count(start_t) == 0 || aom.abs_order_map.count(end_t) == 0) continue;

        ild.imu_meas[kv.first] = &kv.second;
      }

      auto lqr = LinearizationBase<Scalar, POSE_SIZE>::create(this, aom, lqr_options, &marg_data, &ild, &kfs_to_marg,
                                                              &lost_landmaks, last_state_to_marg);

      lqr->linearizeProblem();

      if (show_uimat(UIMAT::JR_M)) visual_data->getj(UIMAT::JR_M).Jr = lqr->getUILandmarkBlocks();

      lqr->performQR();
      if (show_uimat(UIMAT::JR_M_QR)) visual_data->getj(UIMAT::JR_M_QR).Jr = lqr->getUILandmarkBlocks();

      if (is_lin_sqrt && marg_data.is_sqrt) {
        lqr->get_dense_Q2Jp_Q2r(Q2Jp_or_H, Q2r_or_b);
      } else {
        lqr->get_dense_H_b(Q2Jp_or_H, Q2r_or_b);
      }

      stats_sums_.add("marg_linearize", t_linearize.elapsed()).format("ms");
    }

    //    KeypointVioEstimator::linearizeAbsIMU(
    //        aom, accum.getH(), accum.getB(), imu_error, bg_error, ba_error,
    //        frame_states, imu_meas, gyro_bias_weight, accel_bias_weight, g);
    //    linearizeMargPrior(marg_order, marg_sqrt_H, marg_sqrt_b, aom,
    //    accum.getH(),
    //                       accum.getB(), marg_prior_error);

    // Save marginalization prior
    if (out_marg_queue && !kfs_to_marg.empty()) {
      // int64_t kf_id = *kfs_to_marg.begin();

      {
        MargData::Ptr m(new MargData);
        m->aom = aom;

        if (is_lin_sqrt && marg_data.is_sqrt) {
          m->abs_H = (Q2Jp_or_H.transpose() * Q2Jp_or_H).template cast<double>();
          m->abs_b = (Q2Jp_or_H.transpose() * Q2r_or_b).template cast<double>();
        } else {
          m->abs_H = Q2Jp_or_H.template cast<double>();

          m->abs_b = Q2r_or_b.template cast<double>();
        }

        assign_cast_map_values(m->frame_poses, frame_poses);
        assign_cast_map_values(m->frame_states, frame_states);
        m->kfs_all = kf_ids_all;
        m->kfs_to_marg = kfs_to_marg;
        m->use_imu = true;

        for (int64_t t : m->kfs_all) {
          m->opt_flow_res.emplace_back(prev_opt_flow_res.at(t));
        }

        out_marg_queue->push(m);
      }
    }

    // Send a map stamp to the MapDatabase
    if (out_vio_data_queue && !kfs_to_marg.empty()) {
      map_stamp->lmdb = std::make_shared<LandmarkDatabase<Scalar_>>(lmdb);
      out_vio_data_queue->push(map_stamp);
    }

    std::set<int> idx_to_keep, idx_to_marg;
    for (const auto& kv : aom.abs_order_map) {
      if (kv.second.second == POSE_SIZE) {
        int start_idx = kv.second.first;
        if (poses_to_marg.count(kv.first) == 0) {
          for (size_t i = 0; i < POSE_SIZE; i++) idx_to_keep.emplace(start_idx + i);
        } else {
          for (size_t i = 0; i < POSE_SIZE; i++) idx_to_marg.emplace(start_idx + i);
        }
      } else {
        BASALT_ASSERT(kv.second.second == POSE_VEL_BIAS_SIZE);
        // state
        int start_idx = kv.second.first;
        if (states_to_marg_all.count(kv.first) > 0) {
          for (size_t i = 0; i < POSE_VEL_BIAS_SIZE; i++) idx_to_marg.emplace(start_idx + i);
        } else if (states_to_marg_vel_bias.count(kv.first) > 0) {
          for (size_t i = 0; i < POSE_SIZE; i++) idx_to_keep.emplace(start_idx + i);
          for (size_t i = POSE_SIZE; i < POSE_VEL_BIAS_SIZE; i++) idx_to_marg.emplace(start_idx + i);
        } else {
          BASALT_ASSERT(kv.first == last_state_to_marg);
          for (size_t i = 0; i < POSE_VEL_BIAS_SIZE; i++) idx_to_keep.emplace(start_idx + i);
        }
      }
    }

    if (config.vio_debug) {
      std::cout << "keeping " << idx_to_keep.size() << " marg " << idx_to_marg.size() << " total " << asize
                << std::endl;
      std::cout << "last_state_to_marg " << last_state_to_marg << " frame_poses " << frame_poses.size()
                << " frame_states " << frame_states.size() << std::endl;
    }

    if (config.vio_debug || config.vio_extended_logging) {
      MatX Q2Jp_or_H_nullspace;
      VecX Q2r_or_b_nullspace;

      typename LinearizationBase<Scalar, POSE_SIZE>::Options lqr_options;
      lqr_options.lb_options.huber_parameter = huber_thresh;
      lqr_options.lb_options.obs_std_dev = obs_std_dev;
      lqr_options.linearization_type = config.vio_linearization_type;

      nullspace_marg_data.order = marg_data.order;

      ImuLinData<Scalar> ild = {g, gyro_bias_sqrt_weight, accel_bias_sqrt_weight, {}};

      for (const auto& kv : imu_meas) {
        int64_t start_t = kv.second.get_start_t_ns();
        int64_t end_t = kv.second.get_start_t_ns() + kv.second.get_dt_ns();

        if (aom.abs_order_map.count(start_t) == 0 || aom.abs_order_map.count(end_t) == 0) continue;

        ild.imu_meas[kv.first] = &kv.second;
      }

      auto lqr = LinearizationBase<Scalar, POSE_SIZE>::create(this, aom, lqr_options, &nullspace_marg_data, &ild,
                                                              &kfs_to_marg, &lost_landmaks, last_state_to_marg);

      lqr->linearizeProblem();
      lqr->performQR();

      if (is_lin_sqrt && marg_data.is_sqrt) {
        lqr->get_dense_Q2Jp_Q2r(Q2Jp_or_H_nullspace, Q2r_or_b_nullspace);
      } else {
        lqr->get_dense_H_b(Q2Jp_or_H_nullspace, Q2r_or_b_nullspace);
      }

      MatX nullspace_sqrt_H_new;
      VecX nullspace_sqrt_b_new;

      if (is_lin_sqrt && marg_data.is_sqrt) {
        MargHelper<Scalar>::marginalizeHelperSqrtToSqrt(Q2Jp_or_H_nullspace, Q2r_or_b_nullspace, idx_to_keep,
                                                        idx_to_marg, nullspace_sqrt_H_new, nullspace_sqrt_b_new);
      } else if (marg_data.is_sqrt) {
        MargHelper<Scalar>::marginalizeHelperSqToSqrt(Q2Jp_or_H_nullspace, Q2r_or_b_nullspace, idx_to_keep, idx_to_marg,
                                                      nullspace_sqrt_H_new, nullspace_sqrt_b_new);
      } else {
        MargHelper<Scalar>::marginalizeHelperSqToSq(Q2Jp_or_H_nullspace, Q2r_or_b_nullspace, idx_to_keep, idx_to_marg,
                                                    nullspace_sqrt_H_new, nullspace_sqrt_b_new);
      }

      nullspace_marg_data.H = nullspace_sqrt_H_new;
      nullspace_marg_data.b = nullspace_sqrt_b_new;
    }

    MatX marg_H_new;
    VecX marg_b_new;

    {
      Timer t;
      if (is_lin_sqrt && marg_data.is_sqrt) {
        MargHelper<Scalar>::marginalizeHelperSqrtToSqrt(Q2Jp_or_H, Q2r_or_b, idx_to_keep, idx_to_marg, marg_H_new,
                                                        marg_b_new);
      } else if (marg_data.is_sqrt) {
        MargHelper<Scalar>::marginalizeHelperSqToSqrt(Q2Jp_or_H, Q2r_or_b, idx_to_keep, idx_to_marg, marg_H_new,
                                                      marg_b_new);
      } else {
        MargHelper<Scalar>::marginalizeHelperSqToSq(Q2Jp_or_H, Q2r_or_b, idx_to_keep, idx_to_marg, marg_H_new,
                                                    marg_b_new);
      }

      stats_sums_.add("marg_helper", t.elapsed()).format("ms");
    }

    {
      BASALT_ASSERT(frame_states.at(last_state_to_marg).isLinearized() == false);
      frame_states.at(last_state_to_marg).setLinTrue();
    }

    for (const int64_t id : states_to_marg_all) {
      if (visual_data) visual_data->marginalized_idx[id] = frame_idx.at(id);
      frame_states.erase(id);
      imu_meas.erase(id);
      prev_opt_flow_res.erase(id);
    }

    for (const int64_t id : states_to_marg_vel_bias) {
      const PoseVelBiasStateWithLin<Scalar>& state = frame_states.at(id);
      PoseStateWithLin<Scalar> pose(state);

      frame_poses[id] = pose;
      frame_states.erase(id);
      imu_meas.erase(id);
    }

    for (const int64_t id : poses_to_marg) {
      if (visual_data) visual_data->marginalized_idx[id] = frame_idx.at(id);
      frame_poses.erase(id);
      prev_opt_flow_res.erase(id);
    }

    lmdb.removeKeyframes(kfs_to_marg, poses_to_marg, states_to_marg_all);

    if (config.vio_marg_lost_landmarks) {
      for (const auto& lm_id : lost_landmaks) lmdb.removeLandmark(lm_id);
    }

    AbsOrderMap marg_order_new;

    for (const auto& kv : frame_poses) {
      marg_order_new.abs_order_map[kv.first] = std::make_pair(marg_order_new.total_size, POSE_SIZE);

      marg_order_new.total_size += POSE_SIZE;
      marg_order_new.items++;
    }

    {
      marg_order_new.abs_order_map[last_state_to_marg] = std::make_pair(marg_order_new.total_size, POSE_VEL_BIAS_SIZE);
      marg_order_new.total_size += POSE_VEL_BIAS_SIZE;
      marg_order_new.items++;
    }

    marg_data.H = marg_H_new;
    marg_data.b = marg_b_new;
    marg_data.order = marg_order_new;

    if (show_uimat(UIMAT::HB_M)) {
      visual_data->geth(UIMAT::HB_M).H = std::make_shared<Eigen::MatrixXf>(marg_H_new.template cast<float>());
      visual_data->geth(UIMAT::HB_M).b = std::make_shared<Eigen::VectorXf>(marg_b_new.template cast<float>());
      visual_data->geth(UIMAT::HB_M).aom = std::make_shared<AbsOrderMap>(marg_order_new);
    }

    BASALT_ASSERT(size_t(marg_data.H.cols()) == marg_data.order.total_size);

    // Quadratic prior and "delta" of the current state to the original
    // linearization point give cost function
    //
    //    P(x) = 0.5 || J*(delta+x) + r ||^2.
    //
    // For marginalization this has been linearized at x=0 to give
    // linearization
    //
    //    P(x) = 0.5 || J*x + (J*delta + r) ||^2,
    //
    // with Jacobian J and residual J*delta + r.
    //
    // After marginalization, we recover the original form of the
    // prior. We are left with linearization (in sqrt form)
    //
    //    Pnew(x) = 0.5 || Jnew*x + res ||^2.
    //
    // To recover the original form with delta-independent r, we set
    //
    //    Pnew(x) = 0.5 || Jnew*(delta+x) + (res - Jnew*delta) ||^2,
    //
    // and thus rnew = (res - Jnew*delta).

    VecX delta;
    computeDelta(marg_data.order, delta);
    marg_data.b -= marg_data.H * delta;

    if (config.vio_debug || config.vio_extended_logging) {
      VecX delta;
      computeDelta(marg_data.order, delta);
      nullspace_marg_data.b -= nullspace_marg_data.H * delta;
    }

    stats_sums_.add("marg", t_actual_marg.elapsed()).format("ms");

    if (config.vio_debug) {
      std::cout << "marginalizaon done!!" << std::endl;
    }

    if (config.vio_debug || config.vio_extended_logging) {
      Timer t;
      logMargNullspace();
      stats_sums_.add("marg_log", t.elapsed()).format("ms");
    }

    //    std::cout << "new marg prior order" << std::endl;
    //    marg_order.print_order();
  }

  stats_sums_.add("marginalize", t_total.elapsed()).format("ms");

  return true;
}

template <class Scalar_>
bool SqrtKeypointVioEstimator<Scalar_>::optimize() {
  if (config.vio_debug) {
    std::cout << "=================================" << std::endl;
  }

  if (opt_started || frame_states.size() > 4) {
    opt_started = true;

    // harcoded configs
    // bool scale_Jp = config.vio_scale_jacobian && is_qr_solver();
    // bool scale_Jl = config.vio_scale_jacobian && is_qr_solver();

    // timing
    ExecutionStats stats;
    Timer timer_total;
    Timer timer_iteration;

    // construct order of states in linear system --> sort by ascending
    // timestamp
    AbsOrderMap aom;

    for (const auto& kv : frame_poses) {
      aom.abs_order_map[kv.first] = std::make_pair(aom.total_size, POSE_SIZE);

      // Check that we have the same order as marginalization
      BASALT_ASSERT(marg_data.order.abs_order_map.at(kv.first) == aom.abs_order_map.at(kv.first));

      aom.total_size += POSE_SIZE;
      aom.items++;
    }

    for (const auto& kv : frame_states) {
      aom.abs_order_map[kv.first] = std::make_pair(aom.total_size, POSE_VEL_BIAS_SIZE);

      // Check that we have the same order as marginalization
      if (aom.items < marg_data.order.abs_order_map.size())
        BASALT_ASSERT(marg_data.order.abs_order_map.at(kv.first) == aom.abs_order_map.at(kv.first));

      aom.total_size += POSE_VEL_BIAS_SIZE;
      aom.items++;
    }

    // TODO: Check why we get better accuracy with old SC loop. Possible
    // culprits:
    // - different initial lambda (based on previous iteration)
    // - no landmark damping
    // - outlier removal after 4 iterations?
    lambda = Scalar(config.vio_lm_lambda_initial);

    // record stats
    stats.add("num_cams", this->frame_poses.size()).format("count");
    stats.add("num_lms", this->lmdb.numLandmarks()).format("count");
    stats.add("num_obs", this->lmdb.numObservations()).format("count");

    // setup landmark blocks
    typename LinearizationBase<Scalar, POSE_SIZE>::Options lqr_options;
    lqr_options.lb_options.huber_parameter = huber_thresh;
    lqr_options.lb_options.obs_std_dev = obs_std_dev;
    lqr_options.linearization_type = config.vio_linearization_type;

    std::unique_ptr<LinearizationBase<Scalar, POSE_SIZE>> lqr;

    ImuLinData<Scalar> ild = {g, gyro_bias_sqrt_weight, accel_bias_sqrt_weight, {}};
    for (const auto& kv : imu_meas) {
      ild.imu_meas[kv.first] = &kv.second;
    }

    {
      Timer t;
      lqr = LinearizationBase<Scalar, POSE_SIZE>::create(this, aom, lqr_options, &marg_data, &ild);
      stats.add("allocateLMB", t.reset()).format("ms");
      lqr->log_problem_stats(stats);
    }

    bool terminated = false;
    bool converged = false;
    std::string message;

    int it = 0;
    int it_rejected = 0;
    for (; it <= config.vio_max_iterations && !terminated;) {
      if (it > 0) {
        timer_iteration.reset();
      }

      Scalar error_total = 0;
      VecX Jp_column_norm2;

      {
        // TODO: execution could be done staged

        Timer t;

        // linearize residuals
        bool numerically_valid;
        error_total = lqr->linearizeProblem(&numerically_valid);
        if (show_uimat(UIMAT::JR)) visual_data->getj(UIMAT::JR).Jr = lqr->getUILandmarkBlocks();

        if (!numerically_valid) {
          std::cerr << "did not expect numerical failure during linearization\n";
          return false;
        }

        stats.add("linearizeProblem", t.reset()).format("ms");

        //        // compute pose jacobian norm squared for Jacobian scaling
        //        if (scale_Jp) {
        //          Jp_column_norm2 = lqr->getJp_diag2();
        //          stats.add("getJp_diag2", t.reset()).format("ms");
        //        }

        //        // scale landmark jacobians
        //        if (scale_Jl) {
        //          lqr->scaleJl_cols();
        //          stats.add("scaleJl_cols", t.reset()).format("ms");
        //        }

        // marginalize points in place
        lqr->performQR();
        if (show_uimat(UIMAT::JR_QR)) visual_data->getj(UIMAT::JR_QR).Jr = lqr->getUILandmarkBlocks();

        stats.add("performQR", t.reset()).format("ms");
      }

      if (config.vio_debug) {
        // TODO: num_points debug output missing
        std::cout << "[LINEARIZE] Error: " << error_total << " num points " << std::endl;
        std::cout << "Iteration " << it << " " << error_total << std::endl;
      }

      // compute pose jacobian scaling
      //      VecX jacobian_scaling;
      //      if (scale_Jp) {
      //        // TODO: what about jacobian scaling for SC solver?

      //        // ceres uses 1.0 / (1.0 + sqrt(SquaredColumnNorm))
      //        // we use 1.0 / (eps + sqrt(SquaredColumnNorm))
      //        jacobian_scaling = (lqr_options.lb_options.jacobi_scaling_eps +
      //                            Jp_column_norm2.array().sqrt())
      //                               .inverse();
      //      }
      // if (config.vio_debug) {
      //   std::cout << "\t[INFO] Stage 1" << std::endl;
      //}

      // inner loop for backtracking in LM (still count as main iteration
      // though)
      for (int j = 0; it <= config.vio_max_iterations && !terminated; j++) {
        if (j > 0) {
          timer_iteration.reset();
          if (config.vio_debug) {
            std::cout << "Iteration " << it << ", backtracking" << std::endl;
          }
        }

        {
          // Timer t;

          // TODO: execution could be done staged

          //          // set (updated) damping for poses
          //          if (config.vio_lm_pose_damping_variant == 0) {
          //            lqr->setPoseDamping(lambda);
          //            stats.add("setPoseDamping", t.reset()).format("ms");
          //          }

          //          // scale landmark Jacobians only on the first inner
          //          iteration. if (scale_Jp && j == 0) {
          //            lqr->scaleJp_cols(jacobian_scaling);
          //            stats.add("scaleJp_cols", t.reset()).format("ms");
          //          }

          //          // set (updated) damping for landmarks
          //          if (config.vio_lm_landmark_damping_variant == 0) {
          //            lqr->setLandmarkDamping(lambda);
          //            stats.add("setLandmarkDamping", t.reset()).format("ms");
          //          }
        }

        // if (config.vio_debug) {
        //   std::cout << "\t[INFO] Stage 2 " << std::endl;
        // }

        VecX inc;
        {
          Timer t;

          // get dense reduced camera system
          MatX H;
          VecX b;

          lqr->get_dense_H_b(H, b);

          for (const int64_t& ts : ltkfs) {
            if (aom.abs_order_map.count(ts) < 0) {
              printf("UNEXPECTED: ltkf ts=%ld not in aom\n", ts);
              continue;
            }
            const auto& [idx, size] = aom.abs_order_map.at(ts);
            H.template block(idx, 0, size, H.cols()).setZero();
            H.diagonal().template segment<POSE_SIZE>(idx).array() = 1e20;
            b.template segment(idx, size).setZero();
          }

          stats.add("get_dense_H_b", t.reset()).format("ms");

          int iter = 0;
          bool inc_valid = false;
          constexpr int max_num_iter = 3;

          while (iter < max_num_iter && !inc_valid) {
            VecX Hdiag_lambda = (H.diagonal() * lambda).cwiseMax(min_lambda);
            MatX H_copy = H;
            H_copy.diagonal() += Hdiag_lambda;

            Eigen::LDLT<Eigen::Ref<MatX>> ldlt(H_copy);
            inc = ldlt.solve(b);
            stats.add("solve", t.reset()).format("ms");

            if (!inc.array().isFinite().all()) {
              lambda = lambda_vee * lambda;
              lambda_vee *= vee_factor;
            } else {
              inc_valid = true;
            }
            iter++;
          }

          if (!inc_valid) {
            std::cerr << "Still invalid inc after " << max_num_iter << " iterations." << std::endl;
          }

          if (show_uimat(UIMAT::HB)) {
            visual_data->geth(UIMAT::HB).H = std::make_shared<Eigen::MatrixXf>(H.template cast<float>());
            visual_data->geth(UIMAT::HB).b = std::make_shared<Eigen::VectorXf>(b.template cast<float>());
            visual_data->geth(UIMAT::HB).aom = std::make_shared<AbsOrderMap>(aom);
          }
        }

        // backup state (then apply increment and check cost decrease)
        backup();

        // backsubstitute (with scaled pose increment)
        Scalar l_diff = 0;
        {
          // negate pose increment before point update
          inc = -inc;

          Timer t;
          l_diff = lqr->backSubstitute(inc);
          stats.add("backSubstitute", t.reset()).format("ms");
        }

        // undo jacobian scaling before applying increment to poses
        //        if (scale_Jp) {
        //          inc.array() *= jacobian_scaling.array();
        //        }

        // apply increment to poses
        for (auto& [frame_id, state] : frame_poses) {
          int idx = aom.abs_order_map.at(frame_id).first;
          state.applyInc(inc.template segment<POSE_SIZE>(idx));
        }

        for (auto& [frame_id, state] : frame_states) {
          int idx = aom.abs_order_map.at(frame_id).first;
          state.applyInc(inc.template segment<POSE_VEL_BIAS_SIZE>(idx));
        }

        // compute stepsize
        Scalar step_norminf = inc.array().abs().maxCoeff();

        // compute error update applying increment
        Scalar after_update_marg_prior_error = 0;
        Scalar after_update_vision_and_inertial_error = 0;

        {
          Timer t;
          computeError(after_update_vision_and_inertial_error);
          computeMargPriorError(marg_data, after_update_marg_prior_error);

          Scalar after_update_imu_error = 0, after_bg_error = 0, after_ba_error = 0;
          ScBundleAdjustmentBase<Scalar>::computeImuError(
              aom, after_update_imu_error, after_bg_error, after_ba_error, frame_states, imu_meas,
              gyro_bias_sqrt_weight.array().square(), accel_bias_sqrt_weight.array().square(), g);

          after_update_vision_and_inertial_error += after_update_imu_error + after_bg_error + after_ba_error;

          stats.add("computerError2", t.reset()).format("ms");
        }

        Scalar after_error_total = after_update_vision_and_inertial_error + after_update_marg_prior_error;

        // check cost decrease compared to quadratic model cost
        Scalar f_diff;
        bool step_is_valid = false;
        bool step_is_successful = false;
        Scalar relative_decrease = 0;
        {
          // compute actual cost decrease
          f_diff = error_total - after_error_total;

          relative_decrease = f_diff / l_diff;

          if (config.vio_debug) {
            std::cout << "\t[EVAL] error: {:.4e}, f_diff {:.4e} l_diff {:.4e} "
                         "step_quality {:.2e} step_size {:.2e}\n"_format(after_error_total, f_diff, l_diff,
                                                                         relative_decrease, step_norminf);
          }

          // TODO: consider to remove assert. For now we want to test if we
          // even run into the l_diff <= 0 case ever in practice
          // BASALT_ASSERT_STREAM(l_diff > 0, "l_diff " << l_diff);

          // l_diff <= 0 is a theoretical possibility if the model cost change
          // is tiny and becomes numerically negative (non-positive). It might
          // not occur since our linear systems are not that big (compared to
          // large scale BA for example) and we also abort optimization quite
          // early and usually don't have large damping (== tiny step size).
          step_is_valid = l_diff > 0;
          step_is_successful = step_is_valid && relative_decrease > 0;
        }

        double iteration_time = timer_iteration.elapsed();
        double cumulative_time = timer_total.elapsed();

        stats.add("iteration", iteration_time).format("ms");
        {
          basalt::MemoryInfo mi;
          if (get_memory_info(mi)) {
            stats.add("resident_memory", mi.resident_memory);
            stats.add("resident_memory_peak", mi.resident_memory_peak);
          }
        }

        if (step_is_successful) {
          BASALT_ASSERT(step_is_valid);

          if (config.vio_debug) {
            //          std::cout << "\t[ACCEPTED] lambda:" << lambda
            //                    << " Error: " << after_error_total <<
            //                    std::endl;

            std::cout << "\t[ACCEPTED] error: {:.4e}, lambda: {:.1e}, it_time: "
                         "{:.3f}s, total_time: {:.3f}s\n"
                         ""_format(after_error_total, lambda, iteration_time, cumulative_time);
          }

          lambda *= std::max<Scalar>(Scalar(1.0) / 3, 1 - std::pow<Scalar>(2 * relative_decrease - 1, 3));
          lambda = std::max(min_lambda, lambda);

          lambda_vee = initial_vee;

          it++;

          // check function and parameter tolerance
          if ((f_diff > 0 && f_diff < Scalar(1e-6)) || step_norminf < Scalar(1e-4)) {
            converged = true;
            terminated = true;
          }

          // stop inner lm loop
          break;
        } else {
          std::string reason = step_is_valid ? "REJECTED" : "INVALID";

          if (config.vio_debug) {
            //          std::cout << "\t[REJECTED] lambda:" << lambda
            //                    << " Error: " << after_error_total <<
            //                    std::endl;

            std::cout << "\t[{}] error: {}, lambda: {:.1e}, it_time:"
                         "{:.3f}s, total_time: {:.3f}s\n"
                         ""_format(reason, after_error_total, lambda, iteration_time, cumulative_time);
          }

          lambda = lambda_vee * lambda;
          lambda_vee *= vee_factor;

          //        lambda = std::max(min_lambda, lambda);
          //        lambda = std::min(max_lambda, lambda);

          restore();
          it++;
          it_rejected++;

          if (lambda > max_lambda) {
            terminated = true;
            message = "Solver did not converge and reached maximum damping lambda";
          }
        }
      }
    }

    stats.add("optimize", timer_total.elapsed()).format("ms");
    stats.add("num_it", it).format("count");
    stats.add("num_it_rejected", it_rejected).format("count");

    // TODO: call filterOutliers at least once (also for CG version)

    stats_all_.merge_all(stats);
    stats_sums_.merge_sums(stats);

    if (config.vio_debug) {
      if (!converged) {
        if (terminated) {
          std::cout << "Solver terminated early after {} iterations: {}"_format(it, message);
        } else {
          std::cout << "Solver did not converge after maximum number of {} iterations"_format(it);
        }
      }

      stats.print();

      std::cout << "=================================" << std::endl;
    }
  }

  return true;
}

template <class Scalar_>
bool SqrtKeypointVioEstimator<Scalar_>::optimize_and_marg(const OpticalFlowInput::Ptr& input_images,
                                                          const std::map<int64_t, int>& num_points_connected,
                                                          const std::unordered_set<KeypointId>& lost_landmaks) {
  bool success = true;

  success &= optimize();
  if (!success) return false;
  input_images->addTime("optimized");

  success &= marginalize(num_points_connected, lost_landmaks);
  if (!success) return false;
  input_images->addTime("marginalized");

  return success;
}

template <class Scalar_>
void SqrtKeypointVioEstimator<Scalar_>::debug_finalize() {
  std::cout << "=== stats all ===\n";
  stats_all_.print();
  std::cout << "=== stats sums ===\n";
  stats_sums_.print();

  // save files
  stats_all_.save_json("stats_all.json");
  stats_sums_.save_json("stats_sums.json");
}

// //////////////////////////////////////////////////////////////////
// instatiate templates

#ifdef BASALT_INSTANTIATIONS_DOUBLE
template class SqrtKeypointVioEstimator<double>;
#endif

#ifdef BASALT_INSTANTIATIONS_FLOAT
template class SqrtKeypointVioEstimator<float>;
#endif

}  // namespace basalt
