#include <basalt/vi_estimator/map_database.h>

namespace basalt {

MapDatabase::MapDatabase(const VioConfig& config, const Calibration<double>& calib)
    : out_vis_queue(nullptr) {
  this->config = config;
  this->calib = calib;
  this->map = LandmarkDatabase<float>("Persistent Map");
}

void MapDatabase::initialize() {
  auto proc_func = [&]() {
    basalt::MapStamp::Ptr map_stamp;
    while (true) {
      // TODO@brunozanotti: this should be try_pop?
      in_map_stamp_queue.pop(map_stamp);

      if (map_stamp == nullptr) {
        map.print();
        if (out_vis_queue) out_vis_queue->push(nullptr);
        break;
      }

      map.mergeLMDB(map_stamp->lmdb, true);

      if (out_vis_queue) {
        map_visual_data = std::make_shared<MapDatabaseVisualizationData>();
        map_visual_data->t_ns = map_stamp->t_ns;
        computeMapVisualData();
        out_vis_queue->push(map_visual_data);
      }
    }
  };
  processing_thread.reset(new std::thread(proc_func));
}

void MapDatabase::get_map_points(Eigen::aligned_vector<Vec3d>& points, std::vector<int>& ids) {
  points.clear();
  ids.clear();

  for (const auto& tcid_host : map.getHostKfs()) {
    int64_t id = tcid_host.frame_id;
    Sophus::SE3d T_w_i = map.getKeyframePose(id).cast<double>();

    const Sophus::SE3d& T_i_c = calib.T_i_c[tcid_host.cam_id];
    Mat4d T_w_c = (T_w_i * T_i_c).matrix();

    for (const auto& [lm_id, lm_pos] : map.getLandmarksForHostWithIds(tcid_host)) {
      Vec4d pt_cam = StereographicParam<double>::unproject(lm_pos->direction.cast<double>());
      pt_cam[3] = lm_pos->inv_dist;

      Vec4d pt_w = T_w_c * pt_cam;

      points.emplace_back((pt_w.template head<3>() / pt_w[3]).template cast<double>());
      ids.emplace_back(lm_id);
    }
  }
}

Eigen::aligned_map<LandmarkId, Eigen::Matrix<double, 3, 1>> MapDatabase::get_landmarks_3d_pos(
    std::set<LandmarkId> landmarks) {
  Eigen::aligned_map<LandmarkId, Vec3d> landmarks_3d{};

  for (const auto lm_id : landmarks) {
    auto lm = map.getLandmark(lm_id);
    int64_t frame_id = lm.host_kf_id.frame_id;
    Sophus::SE3d T_w_i = map.getKeyframePose(frame_id).cast<double>();

    const Sophus::SE3d& T_i_c = calib.T_i_c[lm.host_kf_id.cam_id];
    Mat4d T_w_c = (T_w_i * T_i_c).matrix();

    Vec4d pt_cam = StereographicParam<double>::unproject(lm.direction.cast<double>());
    pt_cam[3] = lm.inv_dist;

    Vec4d pt_w = T_w_c * pt_cam;

    landmarks_3d.emplace(lm_id, (pt_w.template head<3>() / pt_w[3]).template cast<double>());
  }

  return landmarks_3d;
}

void MapDatabase::computeMapVisualData() {
  // show landmarks
  get_map_points(map_visual_data->landmarks, map_visual_data->landmarks_ids);

  // show keyframes
  for (const auto& [frame_id, pose] : map.getKeyframes()) {
    map_visual_data->keyframe_idx[frame_id] = map.getKeyframeIndex(frame_id);
    map_visual_data->keyframe_poses[frame_id] = pose.template cast<double>();
  }

  // show covisibility
  for (const auto& [tcid_h, target_map] : map.getObservations()) {
    for (const auto& [tcid_t, obs] : target_map) {
      Eigen::Vector3d p1 = map.getKeyframePose(tcid_h.frame_id).template cast<double>().translation();
      Eigen::Vector3d p2 = map.getKeyframePose(tcid_t.frame_id).template cast<double>().translation();
      map_visual_data->covisibility.emplace_back(p1);
      map_visual_data->covisibility.emplace_back(p2);
    }
  }

  // Show observations
  for (const auto& [tcid, obs] : map.getKeyframeObs()) {
    Eigen::Vector3d kf_pos = map.getKeyframePose(tcid.frame_id).template cast<double>().translation();
    auto landmarks_3d = get_landmarks_3d_pos(obs);
    for (const auto& lm_id : obs) {
      map_visual_data->observations[lm_id].emplace_back(kf_pos);
      map_visual_data->observations[lm_id].emplace_back(landmarks_3d[lm_id]);
    }
  }
}

}  // namespace basalt
