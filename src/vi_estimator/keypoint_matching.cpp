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
          if (output_visual_queue) output_visual_queue->push(nullptr);
          output_matching_queue->push(nullptr);
          break;
        }
      // TODO: Implement the matching here
      MatchingVisualizationData::Ptr data(new MatchingVisualizationData);
      Eigen::aligned_vector<Eigen::Vector3d> matched_points;

      matched_points = match_keypoints(curr_frame);

      data->t_ns = curr_frame->t_ns;
      data->points = matched_points;

      if (output_visual_queue) output_visual_queue->push(data);
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
Eigen::aligned_vector<Eigen::Vector3d> KeypointMatching::match_keypoints(OpticalFlowResult::Ptr curr_frame) {
  Eigen::aligned_vector<Eigen::Vector3d> points;
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

        // TODO: with concurrent landmark DB this shouldn't be necessary
        Keypoint<Scalar> kpt_pos;
        try {
          kpt_pos = lmdb.getLandmark(new_kp_id);
        } catch (const std::out_of_range& e) {
          std::cout << "WARNING: Landmark " <<  new_kp_id << " out of range" << std::endl;
          continue;
        }
        Vec4 pt_cam = StereographicParam<Scalar>::unproject(kpt_pos.direction);
        pt_cam[3] = kpt_pos.inv_dist;

        const Sophus::SE3<Scalar>& T_i_c = calib.T_i_c.at(i);
        const auto& T_w_i = curr_frame->predicted_state->T_w_i;
        Mat4 T_w_c = (T_w_i * T_i_c).matrix();
        Vec4 pt_w = T_w_c * pt_cam;

        points.emplace_back((pt_w.head<3>() / pt_w[3]).cast<Scalar>());
      }
    }
  }
  return points;
}


KeypointMatching::Ptr KeypointMatchingFactory::getKeypointMatching(const Calibration<double>& calib, const VioConfig& config, bool use_double) {
  KeypointMatching::Ptr res;
    if (use_double) {
#ifdef BASALT_INSTANTIATIONS_DOUBLE
    // Get the Map instance
    res.reset(new KeypointMatching(calib, config));
#else
    BASALT_LOG_FATAL("Compiled without double support.");
#endif
  } else {
#ifdef BASALT_INSTANTIATIONS_FLOAT
    res.reset(new KeypointMatching(calib, config));
#else
    BASALT_LOG_FATAL("Compiled without float support.");
#endif
  }
  return res;
}

}  // namespace basalt
