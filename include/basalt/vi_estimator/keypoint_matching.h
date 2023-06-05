/**
  License
*/
#pragma once

#include <basalt/optical_flow/optical_flow.h>
#include <basalt/vi_estimator/landmark_database.h>


namespace basalt {


class KeypointMatching {
 public:

  using Scalar = float;

  typedef std::shared_ptr<KeypointMatching> Ptr;

  KeypointMatching(const VioConfig& config)
      : output_matching_queue(nullptr) {
    input_matching_queue.set_capacity(10);
    this->config = config;
}

  tbb::concurrent_bounded_queue<OpticalFlowResult::Ptr> input_matching_queue;
  tbb::concurrent_bounded_queue<OpticalFlowResult::Ptr>* output_matching_queue;

  void initialize();

  void match_keypoints(OpticalFlowResult::Ptr frame);

  bool is_observed(KeypointId kp, OpticalFlowResult::Ptr frame);

  virtual ~KeypointMatching() { maybe_join(); }

  inline void maybe_join() {
    if (processing_thread) {
      processing_thread->join();
      processing_thread.reset();
    }
  }

  // Macro defined in the Eigen library, which is a C++ library for linear algebra.
  // This macro is used to enable memory alignment for instances of a class that contain Eigen types, such as matrices or vectors.
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  // TODO: how to define lmdb for doubles without template this class?
  LandmarkDatabase<Scalar>& lmdb = LandmarkDatabase<Scalar>::getInstance();

 private:

  std::shared_ptr<std::thread> processing_thread;
  VioConfig config;
  int num_matches = 0;

  // timing and stats
  // ExecutionStats stats_all_;
  // ExecutionStats stats_sums_;
};
}  // namespace basalt
