/**
  License
*/
#pragma once

#include <basalt/optical_flow/optical_flow.h>
#include <basalt/vi_estimator/landmark_database.h>


namespace basalt {


class KeypointMatching {
 public:

  typedef std::shared_ptr<KeypointMatching> Ptr;

  KeypointMatching(LandmarkDatabase<float>* lmdb)
      : output_matching_queue(nullptr) {
    input_matching_queue.set_capacity(10);
    this->lmdb = lmdb;
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
  LandmarkDatabase<float>* lmdb;

 private:

  std::shared_ptr<std::thread> processing_thread;

  // timing and stats
  // ExecutionStats stats_all_;
  // ExecutionStats stats_sums_;
};

class KeypointMatchingFactory {
 public:
  static typename KeypointMatching::Ptr getKeypointMatching(bool use_double);
};
}  // namespace basalt
