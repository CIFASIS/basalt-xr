#pragma once

#include <basalt/vi_estimator/landmark_database.h>
#include <basalt/vi_estimator/vio_estimator.h>
#include <Eigen/Dense>
#include <memory>
#include <thread>

namespace basalt {

// TODO: Make an abstract struct SpatialDistribution
template <class Scalar_>
struct SpatialDistributionCube {
 public:
  using Scalar = Scalar_;
  using Vec2 = Eigen::Matrix<Scalar, 2, 1>;
  using Vec3 = Eigen::Matrix<Scalar, 3, 1>;

  SpatialDistributionCube() = default;

  SpatialDistributionCube(std::vector<Vec3> points) {
    Vec3 mean = Vec3::Zero();
    Vec3 variance = Vec3::Zero();

    for (const auto& point : points) {
      mean += point;
    }
    mean /= points.size();

    for (const auto& point : points) {
      Vec3 diff = point - mean;
      variance += (diff.array() * diff.array()).matrix();
    }
    variance /= points.size();
    Cx << mean.x() - sqrt(variance.x()), mean.x() + sqrt(variance.x());
    Cy << mean.y() - sqrt(variance.y()), mean.y() + sqrt(variance.y());
    Cz << mean.z() - sqrt(variance.z()), mean.z() + sqrt(variance.z());
  }

  bool hasOverlap(SpatialDistributionCube<Scalar> sdc) {
    bool overlapX = (Cx[0] <= sdc.Cx[1] && Cx[1] >= sdc.Cx[0]);
    bool overlapY = (Cy[0] <= sdc.Cy[1] && Cy[1] >= sdc.Cy[0]);
    bool overlapZ = (Cz[0] <= sdc.Cz[1] && Cz[1] >= sdc.Cz[0]);

    return overlapX && overlapY && overlapZ;
  }

  Vec2 Cx, Cy, Cz;
};

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

  void handleCovisibilityReq();

  void computeSpatialDistributions(const std::set<TimeCamId>& kfs);

  void computeSTSMap();

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
  tbb::concurrent_bounded_queue<int> in_covi_req_queue;
  tbb::concurrent_bounded_queue<LandmarkDatabase<Scalar>::Ptr>* out_covi_res_queue = nullptr;

 private:
  VioConfig config;
  std::shared_ptr<std::thread> processing_thread;
  MapDatabaseVisualizationData::Ptr map_visual_data;
  LandmarkDatabase<Scalar> map;

  // Covisibility
  Eigen::aligned_map<TimeCamId, SpatialDistributionCube<double>> keyframes_sdc;
  LandmarkDatabase<Scalar>::Ptr sts_map =
      std::make_shared<LandmarkDatabase<Scalar>>("STS Submap");  // spatial-temporal sensitive sub-global map
};
}  // namespace basalt
