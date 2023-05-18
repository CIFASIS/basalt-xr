/**
  License
*/
#pragma once

#include <basalt/optical_flow/optical_flow.h>
#include <basalt/vi_estimator/sc_ba_base.h>


namespace basalt {


class KeypointMatching : public BundleAdjustmentBase<float> {
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

 private:

  std::shared_ptr<std::thread> processing_thread;
  VioConfig config;
  int num_matches = 0;

  // timing and stats
  // ExecutionStats stats_all_;
  // ExecutionStats stats_sums_;
};

class KeypointMatchingFactory {
 public:
  static typename KeypointMatching::Ptr getKeypointMatching(const VioConfig& config, bool use_double);
};
}  // namespace basalt
