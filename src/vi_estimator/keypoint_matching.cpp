/**
  License
*/

#include <basalt/vi_estimator/keypoint_matching.h>
#include <tbb/parallel_for.h>

namespace basalt {

/**
 * Initialize the keypoint matching process by creating a processing thread.
 * The processing thread continuously retrieves frames from the Optical Flow output queue,
 * matches keypoints in each frame, and pushes the processed frames into the output matching queue.
 *
 */
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
    std::cout << "Finished matching points. Total matches: " << num_matches << std::endl;
  };
  processing_thread.reset(new std::thread(proc_func));
}

/**
 * Match keypoints in the current frame with keypoints in the landmark database.
 *
 * @param curr_frame A pointer to the current frame containing observations and descriptors.
 */
void KeypointMatching::match_keypoints(OpticalFlowResult::Ptr curr_frame) {

  int NUM_CAMS = curr_frame->observations.size();
  for (int i=0; i < NUM_CAMS; i++) {
    std::vector<Descriptor> descr1;
    std::vector<KeypointId> kp1;
    std::vector<Descriptor> descr2;
    std::vector<KeypointId> kp2;

    // TODO: filter points already matched by optical flow
    for (const auto& ds : curr_frame->descriptors.at(i)) {
      kp1.push_back(ds.first);
      descr1.push_back(ds.second);
    }

    // TODO: filter points already matched by optical flow
    for (const auto& kv : lmdb.getLandmarks()) {
      kp2.push_back(kv.first);
      descr2.push_back(kv.second.descriptor);
    }

    std::vector<std::pair<int, int>> matches;

    matchDescriptors(descr1, descr2, matches,
                      config.mapper_max_hamming_distance,
                      config.mapper_second_best_test_ratio);

    for (const auto& match: matches) {
      // If match: keypoint kp1[i] is the same as kp2[j] so change the kp_id
      KeypointId kp_id = kp1[match.first];
      KeypointId new_kp_id = kp2[match.second];
      // TODO: if we filter the klf matches this shouldn't be necessary
      if (new_kp_id != kp_id) {
        // TODO: check if this is necessary
        if (curr_frame->observations.at(i).count(kp_id) == 0 || curr_frame->observations.at(i).count(new_kp_id) > 0) {
          continue;
        }
        curr_frame->observations.at(i)[new_kp_id] = curr_frame->observations.at(i).at(kp_id);
        curr_frame->descriptors.at(i)[new_kp_id] = curr_frame->descriptors.at(i).at(kp_id);
        curr_frame->observations.at(i).erase(kp_id);
        curr_frame->descriptors.at(i).erase(kp_id);
        num_matches++;
      }
    }
  }
}
}  // namespace basalt
