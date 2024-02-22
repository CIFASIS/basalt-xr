#pragma once

#include <basalt/vi_estimator/landmark_database.h>
#include <basalt/vi_estimator/vio_estimator.h>
#include <Eigen/Dense>
#include <memory>
#include <thread>

namespace basalt {

struct MapDatabaseVisualizationData {
  using Ptr = std::shared_ptr<MapDatabaseVisualizationData>;

  int64_t t_ns;

  Eigen::aligned_vector<Eigen::Vector3d> landmarks;
  std::vector<int> landmarks_ids;
  Eigen::aligned_map<FrameId, size_t> keyframe_idx;
  Eigen::aligned_map<int64_t, Sophus::SE3d> keyframe_poses;
  Eigen::aligned_vector<Eigen::Vector3d> covisibility;
  std::map<int, Eigen::aligned_vector<Eigen::Vector3d>> observations;
};

// TODO: This should be a templated class template <class Scalar>
class MapDatabase {
 public:
  using Scalar = float;
  using Ptr = std::shared_ptr<MapDatabase>;

  using Vec3d = Eigen::Matrix<double, 3, 1>;
  using Vec4d = Eigen::Matrix<double, 4, 1>;
  using Mat4d = Eigen::Matrix<double, 4, 4>;

  using Vec3 = Eigen::Matrix<Scalar, 3, 1>;

  MapDatabase(const VioConfig& config, const basalt::Calibration<double>& calib);

  ~MapDatabase() { maybe_join(); }

  void initialize();

  void get_map_points(Eigen::aligned_vector<Vec3d>& points, std::vector<int>& ids);

  Eigen::aligned_map<LandmarkId, Vec3d> get_landmarks_3d_pos(std::set<LandmarkId> landmarks);

  void computeMapVisualData();
  inline void maybe_join() {
    if (processing_thread) {
      processing_thread->join();
      processing_thread.reset();
    }
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  basalt::Calibration<double> calib;

  tbb::concurrent_bounded_queue<MapStamp::Ptr> in_map_stamp_queue;
  tbb::concurrent_bounded_queue<MapDatabaseVisualizationData::Ptr>* out_vis_queue = nullptr;

 private:
  VioConfig config;
  std::shared_ptr<std::thread> processing_thread;
  MapDatabaseVisualizationData::Ptr map_visual_data;
  LandmarkDatabase<Scalar> map;

};
}  // namespace basalt
