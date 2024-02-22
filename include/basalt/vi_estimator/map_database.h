#pragma once

#include <basalt/vi_estimator/landmark_database.h>
#include <basalt/vi_estimator/vio_estimator.h>
#include <Eigen/Dense>
#include <memory>
#include <thread>

namespace basalt {

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
  inline void maybe_join() {
    if (processing_thread) {
      processing_thread->join();
      processing_thread.reset();
    }
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  basalt::Calibration<double> calib;

  tbb::concurrent_bounded_queue<MapStamp::Ptr> in_map_stamp_queue;

 private:
  VioConfig config;
  std::shared_ptr<std::thread> processing_thread;
  LandmarkDatabase<Scalar> map;

};
}  // namespace basalt
