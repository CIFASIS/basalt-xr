/**
  License
*/

#include <basalt/vi_estimator/keypoint_matching.h>

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
      output_matching_queue->push(curr_frame);
    }
    std::cout << "Finished matching points" << std::endl;
  };
  processing_thread.reset(new std::thread(proc_func));
}

KeypointMatching::Ptr KeypointMatchingFactory::getKeypointMatching() {
  KeypointMatching::Ptr res;
  res.reset(new KeypointMatching());
  return res;
}

}  // namespace basalt
