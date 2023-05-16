/**
  License
*/

#include <basalt/vi_estimator/keypoint_matching.h>
#include <tbb/parallel_for.h>

namespace basalt {

// This functions matches the points not detected by the Optical Flow that has already been stored in the map database
void KeypointMatching::initialize() {
  auto proc_func = [&] {
    std::cout << "Matching points..." << std::endl;
    OpticalFlowResult::Ptr curr_frame;
    while (true) {
      input_matching_queue.pop(curr_frame);
      if (curr_frame == nullptr) {
          output_matching_queue->push(nullptr);
          break;
        }
      // TODO: Implement the matching here
      match_keypoints(curr_frame);
      output_matching_queue->push(curr_frame);
    }
    std::cout << "Finished matching points" << std::endl;
  };
  processing_thread.reset(new std::thread(proc_func));
}

/**
 * For each keypoint in the landmark database that is not observed by optical flow in the
 * current frame, project it onto the frame and try to find it.
 * If a match is found, add it to the current frame's observations.
 *
 * @param curr_frame a shared pointer to the current frame's OpticalFlowResult
 */
void KeypointMatching::match_keypoints(OpticalFlowResult::Ptr curr_frame) {

  // TODO: parallel loop
  for (auto &[kp_id, kp] : lmdb.getLandmarks()) {
    bool observed = is_observed(kp_id, curr_frame);
    if (!observed) {
      // This keypoint wasn't observed by optical flow
      // Project it onto the frame and try to find it.
    }
  }
}

/**
 * Check if a keypoint is observed in a given frame.
 *
 * @param kp_id The ID of the keypoint to search for.
 * @param frame A shared pointer to an OpticalFlowResult object representing the frame to search in.
 * @return True if the keypoint is observed in the frame, false otherwise.
 */
bool KeypointMatching::is_observed(KeypointId kp_id, OpticalFlowResult::Ptr frame) {
  bool found = false;
  tbb::parallel_for(tbb::blocked_range<size_t>(0, frame->observations.size()),
    [&](const tbb::blocked_range<size_t>& r) {
      for (size_t i = r.begin(); i != r.end(); ++i) {
        const auto& observation = frame->observations[i];
        if (observation.find(kp_id) != observation.end()) {
          found = true;
          return;
        }
      }
    });
  return found;
}


KeypointMatching::Ptr KeypointMatchingFactory::getKeypointMatching(bool use_double) {
  KeypointMatching::Ptr res;
    if (use_double) {
#ifdef BASALT_INSTANTIATIONS_DOUBLE
    // Get the Map instance
    res.reset(new KeypointMatching());
#else
    BASALT_LOG_FATAL("Compiled without double support.");
#endif
  } else {
#ifdef BASALT_INSTANTIATIONS_FLOAT
    res.reset(new KeypointMatching());
#else
    BASALT_LOG_FATAL("Compiled without float support.");
#endif
  }
  return res;
}

}  // namespace basalt
