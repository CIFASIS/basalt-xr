/**
  License
*/
#pragma once

// #include <thread>
#include <basalt/optical_flow/optical_flow.h>


namespace basalt {

// we need a template here?
// template <class Scalar_>
// class KeypointMatching : public BundleAdjustmentBase<Scalar> {
class KeypointMatching {
 public:

  typedef std::shared_ptr<KeypointMatching> Ptr;

  KeypointMatching()
      : output_matching_queue(nullptr) {
    input_matching_queue.set_capacity(10);
  }

  tbb::concurrent_bounded_queue<OpticalFlowResult::Ptr> input_matching_queue;
  tbb::concurrent_bounded_queue<OpticalFlowResult::Ptr>* output_matching_queue;

  void initialize();

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
  // using BundleAdjustmentBase<Scalar>::lmdb;

 private:

  std::shared_ptr<std::thread> processing_thread;

  // timing and stats
  // ExecutionStats stats_all_;
  // ExecutionStats stats_sums_;
};

class KeypointMatchingFactory {
 public:
  static typename KeypointMatching::Ptr getKeypointMatching();
};
}  // namespace basalt
