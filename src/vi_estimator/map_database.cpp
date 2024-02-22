#include <basalt/vi_estimator/map_database.h>

namespace basalt {

MapDatabase::MapDatabase(const VioConfig& config, const Calibration<double>& calib) {
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
        break;
      }

      map.mergeLMDB(map_stamp->lmdb, true);
    }
  };
  processing_thread.reset(new std::thread(proc_func));
}
}  // namespace basalt
