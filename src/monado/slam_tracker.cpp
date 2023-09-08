// Copyright 2022, Collabora, Ltd.

#include "slam_tracker.hpp"
#include "slam_tracker_ui.hpp"

#include <pangolin/display/image_view.h>
#include <pangolin/pangolin.h>

#include <CLI/CLI.hpp>

#include <chrono>
#include <cstdio>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_set>
#include "sophus/se3.hpp"

#include <basalt/io/marg_data_io.h>
#include <basalt/serialization/headers_serialization.h>
#include <basalt/vi_estimator/vio_estimator.h>
#include "basalt/utils/vis_utils.h"

#if defined(_WIN32) || defined(__CYGWIN__)
#define EXPORT __declspec(dllexport)
#else
#define EXPORT __attribute__((visibility("default")))
#endif

namespace xrt::auxiliary::tracking::slam {

EXPORT extern const int IMPLEMENTATION_VERSION_MAJOR = HEADER_VERSION_MAJOR;
EXPORT extern const int IMPLEMENTATION_VERSION_MINOR = HEADER_VERSION_MINOR;
EXPORT extern const int IMPLEMENTATION_VERSION_PATCH = HEADER_VERSION_PATCH;

using std::cout;
using std::make_shared;
using std::make_unique;
using std::pair;
using std::shared_ptr;
using std::static_pointer_cast;
using std::string;
using std::thread;
using std::to_string;
using std::unordered_set;
using std::vector;
using namespace basalt;

static const vector<string> timing_titles{
    "frame_ts",
    "tracker_received",
    "tracker_pushed",
    "frames_received",  // < Basalt computation starts
    "opticalflow_produced",
    "vio_start",
    "imu_preintegrated",
    "landmarks_updated",
    "optimized",
    "marginalized",
    "pose_produced",  // Basalt computation ends
    "tracker_consumer_received",
    "tracker_consumer_pushed",
    "monado_dequeued",
};

struct slam_tracker::implementation {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  // Options parsed from unified config file
  bool show_gui = false;
  string cam_calib_path;
  string config_path;
  string marg_data_path;
  bool print_queue = false;
  bool use_double = false;

  // VIO members
  struct {
    bool imu = false;
    vector<bool> cam;
  } calib_data_ready;
  int cam_count;
  Calibration<double> calib;
  VioConfig vio_config;
  OpticalFlowBase::Ptr opt_flow_ptr;
  VioEstimatorBase::Ptr vio;
  int expecting_frame = 0;

  // Queues
  std::atomic<bool> running = false;
  tbb::concurrent_bounded_queue<PoseVelBiasState<double>::Ptr> out_state_queue;
  tbb::concurrent_bounded_queue<PoseVelBiasState<double>::Ptr> monado_out_state_queue;
  tbb::concurrent_bounded_queue<OpticalFlowInput::Ptr> *image_data_queue = nullptr;  // Invariant: not null after ctor
  tbb::concurrent_bounded_queue<ImuData<double>::Ptr> *imu_data_queue = nullptr;     // Invariant: not null after ctor

  // Threads
  thread state_consumer_thread;
  thread queues_printer_thread;

  // External Queues
  slam_tracker_ui ui{};
  MargDataSaver::Ptr marg_data_saver;

  // slam_tracker features
  unordered_set<int> supported_features{
      F_ADD_CAMERA_CALIBRATION,   F_ADD_IMU_CALIBRATION, F_ENABLE_POSE_EXT_TIMING,
      F_ENABLE_POSE_EXT_FEATURES, F_RESET_TRACKER_STATE,
  };

  // Additional calibration data
  vector<cam_calibration> added_cam_calibs{};
  vector<imu_calibration> added_imu_calibs{};

  // Enabled pose extensions
  bool pose_timing_enabled = false;
  bool pose_features_enabled = false;

 public:
  implementation(const slam_config &config) {
    cam_count = config.cam_count;
    show_gui = config.show_ui;
    cout << "Basalt with cam_count=" << cam_count << ", show_gui=" << show_gui << "\n";

    // Basalt in its current state does not support monocular cameras, although it
    // should be possible to adapt it to do so, see:
    // https://gitlab.com/VladyslavUsenko/basalt/-/issues/2#note_201965760
    // https://gitlab.com/VladyslavUsenko/basalt/-/issues/25#note_362741510
    // https://github.com/DLR-RM/granite
    ASSERT(cam_count > 1, "Basalt doesn't support running with %d cameras", cam_count);
    calib_data_ready.cam.resize(cam_count, false);

    if (!config.config_file) {
      // For the pipeline to work now, the user will need to use add_cam/imu_calibration
      return;
    }

    load_unified_config(*config.config_file);

    vio_config.load(config_path);
    load_calibration_data(cam_calib_path);
  }

 private:
  void load_unified_config(const string &unified_config) {
    CLI::App app{"Options for the Basalt SLAM Tracker"};

    app.add_option("--show-gui", show_gui, "Show GUI");
    app.add_option("--cam-calib", cam_calib_path, "Ground-truth camera calibration used for simulation.")->required();
    app.add_option("--config-path", config_path, "Path to config file.")->required();
    app.add_option("--marg-data", marg_data_path, "Path to folder where marginalization data will be stored.");
    app.add_option("--print-queue", print_queue, "Poll and print for queue sizes.");
    app.add_option("--use-double", use_double, "Whether to use a double or single precision pipeline.");

    try {
      // While --config-path sets the VIO configuration, --config sets the
      // entire unified Basalt configuration, including --config-path
      app.set_config("--config", unified_config, "Configuration file.", true);
      app.allow_config_extras(false);  // Makes parsing fail on unknown options
      string unique_config = "--config=" + unified_config;
      app.parse(unique_config);
    } catch (const CLI::ParseError &e) {
      app.exit(e);
      ASSERT(false, "Config file error (%s)", unified_config.c_str());
    }

    cout << "Instantiating Basalt SLAM tracker\n";
    cout << "Using config file: " << app["--config"]->as<string>() << "\n";
    cout << app.config_to_str(true, true) << "\n";
  }

  void load_calibration_data(const string &calib_path) {
    std::ifstream os(calib_path, std::ios::binary);
    if (os.is_open()) {
      cereal::JSONInputArchive archive(os);
      archive(calib);
      int calib_cam_count = calib.intrinsics.size();
      cout << "Loaded camera with " << calib_cam_count << " cameras\n";
      ASSERT_(calib_cam_count == cam_count);
      calib_data_ready.imu = true;
      for (int i = 0; i < calib_cam_count; i++) {
        calib_data_ready.cam.at(i) = true;
      }
    } else {
      std::cerr << "could not load camera calibration " << calib_path << "\n";
      std::abort();
    }
  }

  pose get_pose_from_state(const PoseVelBiasState<double>::Ptr &state) const {
    pose p;
    Sophus::SE3d T_w_i = state->T_w_i;
    p.px = T_w_i.translation().x();
    p.py = T_w_i.translation().y();
    p.pz = T_w_i.translation().z();
    p.rx = T_w_i.unit_quaternion().x();
    p.ry = T_w_i.unit_quaternion().y();
    p.rz = T_w_i.unit_quaternion().z();
    p.rw = T_w_i.unit_quaternion().w();
    p.timestamp = state->t_ns;
    p.next = nullptr;

    auto *next = &p.next;

    if (state->input_images->stats.timing_enabled) {
      pose_ext_timing_data petd = state->input_images->stats;
      auto pose_timing = make_shared<pose_ext_timing>(petd);
      *next = pose_timing;
      next = &pose_timing->next;
    }

    if (state->input_images->stats.features_enabled) {
      pose_ext_features_data pefd = state->input_images->stats;
      auto pose_features = make_shared<pose_ext_features>(pefd);
      *next = pose_features;
      next = &pose_features->next;
    }

    return p;
  }

  void state_consumer() {
    PoseVelBiasState<double>::Ptr data;
    PoseVelBiasState<double>::Ptr _;

    while (true) {
      out_state_queue.pop(data);
      if (data.get() == nullptr) {
        while (!monado_out_state_queue.try_push(nullptr)) monado_out_state_queue.pop(_);
        break;
      }
      data->input_images->addTime("tracker_consumer_received");

      if (show_gui) ui.log_vio_data(data);

      data->input_images->addTime("tracker_consumer_pushed");
      while (!monado_out_state_queue.try_push(data)) monado_out_state_queue.pop(_);
    }

    cout << "Finished state_consumer\n";
  }

  void queues_printer() {
    while (running) {
      cout << "[in] frames: " << image_data_queue->size() << "/" << image_data_queue->capacity() << " \n"
           << "[in] imu: " << imu_data_queue->size() << "/" << imu_data_queue->capacity() << " \n"
           << "[in] depth: " << opt_flow_ptr->input_depth_queue.unsafe_size() << "/-- \n"
           << "[mid] keypoints: " << opt_flow_ptr->output_queue->size() << "/" << opt_flow_ptr->output_queue->capacity()
           << " \n"
           << "[mid] pose: " << out_state_queue.size() << "/" << out_state_queue.capacity() << "\n"
           << "[out] monado queue: " << monado_out_state_queue.size() << "/" << monado_out_state_queue.capacity()
           << "\n"
           << "[out] ui: " << vio->out_vis_queue->size() << "/" << vio->out_vis_queue->capacity() << "\n";
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    cout << "Finished queues_printer\n";
  }

  void print_calibration() {
    std::stringstream ss{};
    cereal::JSONOutputArchive write_to_stream(ss);
    write_to_stream(calib);
    cout << "Calibration: " << ss.str() << "\n";
  }

 public:
  void initialize() {
    // Overwrite camera calibration data
    for (const auto &c : added_cam_calibs) {
      apply_cam_calibration(c);
    }

    // Overwrite IMU calibration data
    for (const auto &c : added_imu_calibs) {
      apply_imu_calibration(c);
    }

    ASSERT(calib_data_ready.imu, "Missing IMU calibration");
    for (size_t i = 0; i < calib_data_ready.cam.size(); i++) {
      ASSERT(calib_data_ready.cam.at(i), "Missing cam%zu calibration", i);
    }

    monado_out_state_queue.set_capacity(32);

    // NOTE: This factory also starts the optical flow
    opt_flow_ptr = OpticalFlowFactory::getOpticalFlow(vio_config, calib);
    opt_flow_ptr->start();
    image_data_queue = &opt_flow_ptr->input_img_queue;
    ASSERT_(image_data_queue != nullptr);

    vio = VioEstimatorFactory::getVioEstimator(vio_config, calib, constants::g, true, use_double);
    imu_data_queue = &vio->imu_data_queue;
    ASSERT_(imu_data_queue != nullptr);

    opt_flow_ptr->output_queue = &vio->vision_data_queue;
    opt_flow_ptr->show_gui = show_gui;
    if (show_gui) {
      ui.initialize(cam_count);
      vio->out_vis_queue = &ui.out_vis_queue;
    };
    vio->out_state_queue = &out_state_queue;
    vio->opt_flow_depth_guess_queue = &opt_flow_ptr->input_depth_queue;
    vio->opt_flow_state_queue = &opt_flow_ptr->input_state_queue;

    if (!marg_data_path.empty()) {
      marg_data_saver.reset(new MargDataSaver(marg_data_path));
      vio->out_marg_queue = &marg_data_saver->in_marg_queue;
    }
  }

  void start() {
    running = true;
    vio->initialize(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    if (show_gui) ui.start(vio->getT_w_i_init(), calib, vio_config, &opt_flow_ptr->input_depth_queue, opt_flow_ptr);
    state_consumer_thread = thread(&slam_tracker::implementation::state_consumer, this);
    if (print_queue) queues_printer_thread = thread(&slam_tracker::implementation::queues_printer, this);
  }

  void stop() {
    running = false;
    image_data_queue->push(nullptr);
    imu_data_queue->push(nullptr);
    opt_flow_ptr->input_imu_queue.push(nullptr);

    if (print_queue) queues_printer_thread.join();
    state_consumer_thread.join();
    if (show_gui) ui.stop();

    // TODO: There is a segfault when closing monado without starting the stream
    // happens in a lambda from keypoint_vio.cpp and ends at line calib_bias.hpp:112
  }

  void finalize() {
    // Only the OpticalFlow gets started by initialize, finish it with this
    image_data_queue->push(nullptr);
  }

  bool is_running() { return running; }

  void push_imu_sample(const imu_sample &s) {
    // concurrent_bounded_queue expects Erasable and Allocator named
    // requirements for the type, using a pointer because it already is. This is
    // done in the others examples as well but it is far from optimal.
    ImuData<double>::Ptr data;
    data.reset(new ImuData<double>);
    data->t_ns = s.timestamp;
    data->accel = {s.ax, s.ay, s.az};
    data->gyro = {s.wx, s.wy, s.wz};
    imu_data_queue->push(data);
    opt_flow_ptr->input_imu_queue.push(data);
  }

 private:
  OpticalFlowInput::Ptr partial_frame;

 public:
  void push_frame(const img_sample &s) {
    int i = s.cam_index;
    ASSERT(expecting_frame == i, "Expected cam%d frame, received cam%d", expecting_frame, i);

    expecting_frame = (expecting_frame + 1) % cam_count;

    if (i == 0) {
      partial_frame = make_shared<OpticalFlowInput>(cam_count);

      partial_frame->t_ns = s.timestamp;

      // Initialize stats
      partial_frame->stats.ts = s.timestamp;
      if (pose_timing_enabled) {
        partial_frame->stats.timing_enabled = true;
        partial_frame->stats.timing.reserve(timing_titles.size());
        partial_frame->stats.timing_titles = &timing_titles;
        partial_frame->addTime("frame_ts", s.timestamp);
        partial_frame->addTime("tracker_received");
      }
      if (pose_features_enabled) {
        partial_frame->stats.features_enabled = true;
        partial_frame->stats.features_per_cam.resize(cam_count);
      }

    } else {
      ASSERT(partial_frame->t_ns == s.timestamp, "cam0 and cam%d frame timestamps differ: %ld != %ld", i,
             partial_frame->t_ns, s.timestamp);
    }

    int width = s.img.cols;
    int height = s.img.rows;
    // Forced to use uint16_t here, in place because of cameras with 12-bit grayscale support
    auto &mimg = partial_frame->img_data[i].img;
    mimg.reset(new ManagedImage<uint16_t>(width, height));

    for (size_t j = 0; j < s.masks.size(); j++) {
      auto &r = s.masks[j];
      partial_frame->masks[i].masks.emplace_back(r.x, r.y, r.w, r.h);
    }

    // TODO: We could avoid this copy. Maybe by writing a custom
    // allocator for ManagedImage that ties the OpenCV allocator
    size_t full_size = width * height;
    for (size_t j = 0; j < full_size; j++) {
      mimg->ptr[j] = s.img.at<uchar>(j) << 8;
    }

    if (i == cam_count - 1) {
      partial_frame->addTime("tracker_pushed");
      image_data_queue->push(partial_frame);
      if (show_gui) ui.update_last_image(partial_frame);
    }
  }

  bool try_dequeue_pose(pose &p) {
    PoseVelBiasState<double>::Ptr state;
    bool dequeued = monado_out_state_queue.try_pop(state);
    if (dequeued) {
      state->input_images->addTime("monado_dequeued");
      p = get_pose_from_state(state);
    }
    return dequeued;
  }

  bool supports_feature(int feature_id) { return supported_features.count(feature_id) == 1; }

  bool use_feature(int feature_id, const shared_ptr<void> &params, shared_ptr<void> &result) {
    result = nullptr;
    if (feature_id == FID_ACC) {
      shared_ptr<FPARAMS_ACC> casted_params = static_pointer_cast<FPARAMS_ACC>(params);
      add_cam_calibration(*casted_params);
    } else if (feature_id == FID_AIC) {
      shared_ptr<FPARAMS_AIC> casted_params = static_pointer_cast<FPARAMS_AIC>(params);
      add_imu_calibration(*casted_params);
    } else if (feature_id == FID_EPET) {
      shared_ptr<FPARAMS_EPET> casted_params = static_pointer_cast<FPARAMS_EPET>(params);
      result = enable_pose_ext_timing(*casted_params);
    } else if (feature_id == FID_EPEF) {
      shared_ptr<FPARAMS_EPEF> casted_params = static_pointer_cast<FPARAMS_EPEF>(params);
      enable_pose_ext_features(*casted_params);
    } else if (feature_id == FID_RS) {
      reset_tracker_state();
    } else {
      return false;
    }
    return true;
  }

  void add_cam_calibration(const cam_calibration &cam_calib) { added_cam_calibs.push_back(cam_calib); }

  void apply_cam_calibration(const cam_calibration &cam_calib) {
    using Scalar = double;
    size_t i = cam_calib.cam_index;

    const auto &tic = cam_calib.t_imu_cam;
    Eigen::Matrix3d ric;
    ric << tic(0, 0), tic(0, 1), tic(0, 2), tic(1, 0), tic(1, 1), tic(1, 2), tic(2, 0), tic(2, 1), tic(2, 2);
    Eigen::Quaterniond q(ric);
    Eigen::Vector3d p{tic(0, 3), tic(1, 3), tic(2, 3)};
    ASSERT_(calib.T_i_c.size() == i);
    calib.T_i_c.push_back(Calibration<Scalar>::SE3(q, p));

    GenericCamera<double> model;
    const vector<Scalar> &d = cam_calib.distortion;
    if (cam_calib.distortion_model == "none") {
      ASSERT_(d.size() == 0);
      PinholeCamera<Scalar>::VecN mp;
      mp << cam_calib.fx, cam_calib.fy, cam_calib.cx, cam_calib.cy;
      PinholeCamera pinhole(mp);
      model.variant = pinhole;
    } else if (cam_calib.distortion_model == "kb4") {
      ASSERT_(d.size() == 4);
      KannalaBrandtCamera4<Scalar>::VecN mp;
      mp << cam_calib.fx, cam_calib.fy, cam_calib.cx, cam_calib.cy, d[0], d[1], d[2], d[3];
      KannalaBrandtCamera4 kannala_brandt(mp);
      model.variant = kannala_brandt;
    } else if (cam_calib.distortion_model == "rt8") {
      ASSERT_(d.size() == 9);  // 8 and rpmax
      PinholeRadtan8Camera<Scalar>::VecN mp;
      mp << cam_calib.fx, cam_calib.fy, cam_calib.cx, cam_calib.cy, d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[7];
      Scalar rpmax = d[8];
      PinholeRadtan8Camera pinhole_radtan8(mp, rpmax);
      model.variant = pinhole_radtan8;
    } else {
      ASSERT(false, "Unsupported camera model (%s)", cam_calib.distortion_model.c_str());
    }
    ASSERT_(calib.intrinsics.size() == i);
    calib.intrinsics.push_back(model);

    ASSERT_(calib.resolution.size() == i);
    calib.resolution.push_back({cam_calib.width, cam_calib.height});

    calib_data_ready.cam.at(i) = true;
  }

  void add_imu_calibration(const imu_calibration &imu_calib) { added_imu_calibs.push_back(imu_calib); }

  void apply_imu_calibration(const imu_calibration &imu_calib) {
    using Scalar = double;

    int i = imu_calib.imu_index;
    ASSERT(i == 0, "More than one IMU unsupported (%d)", i);

    static double frequency = -1;
    if (frequency == -1) {
      frequency = imu_calib.frequency;
      calib.imu_update_rate = frequency;
    } else {
      ASSERT(frequency == calib.imu_update_rate, "Unsupported mix of IMU frequencies %lf != %lf", frequency,
             calib.imu_update_rate);
    }

    // Accelerometer calibration

    inertial_calibration accel = imu_calib.accel;

    Eigen::Matrix<Scalar, 9, 1> accel_bias_full;
    const auto &abias = accel.offset;
    const auto &atran = accel.transform;

    // TODO: Doing the same as rs_t265.cpp but that's incorrect. We should be doing an LQ decomposition of atran and
    // using L. See https://gitlab.com/VladyslavUsenko/basalt-headers/-/issues/8
    accel_bias_full << abias(0), abias(1), abias(2), atran(0, 0) - 1, atran(1, 0), atran(2, 0), atran(1, 1) - 1,
        atran(2, 1), atran(2, 2) - 1;
    CalibAccelBias<Scalar> accel_bias;
    accel_bias.getParam() = accel_bias_full;
    calib.calib_accel_bias = accel_bias;

    calib.accel_noise_std = {accel.noise_std(0), accel.noise_std(1), accel.noise_std(2)};
    calib.accel_bias_std = {accel.bias_std(0), accel.bias_std(1), accel.bias_std(2)};

    // Gyroscope calibration

    inertial_calibration gyro = imu_calib.gyro;

    Eigen::Matrix<Scalar, 12, 1> gyro_bias_full;
    const auto &gbias = gyro.offset;
    const auto &gtran = gyro.transform;
    gyro_bias_full << gbias(0), gbias(1), gbias(2), gtran(0, 0) - 1, gtran(1, 0), gtran(2, 0), gtran(0, 1),
        gtran(1, 1) - 1, gtran(2, 1), gtran(0, 2), gtran(1, 2), gtran(2, 2) - 1;
    CalibGyroBias<Scalar> gyro_bias;
    gyro_bias.getParam() = gyro_bias_full;
    calib.calib_gyro_bias = gyro_bias;

    calib.gyro_noise_std = {gyro.noise_std(0), gyro.noise_std(1), gyro.noise_std(2)};
    calib.gyro_bias_std = {gyro.bias_std(0), gyro.bias_std(1), gyro.bias_std(2)};

    calib_data_ready.imu = true;
  }

  shared_ptr<vector<string>> enable_pose_ext_timing(bool enable) {
    pose_timing_enabled = enable;
    return make_shared<vector<string>>(timing_titles);
  }

  void enable_pose_ext_features(bool enable) { pose_features_enabled = enable; }

  void reset_tracker_state() {
    std::cout << "Tracker state reset\n";
    vio->scheduleResetState();
  }
};

EXPORT slam_tracker::slam_tracker(const slam_config &slam_config) {
  impl = make_unique<slam_tracker::implementation>(slam_config);
}

EXPORT slam_tracker::~slam_tracker() = default;

EXPORT void slam_tracker::initialize() { impl->initialize(); }

EXPORT void slam_tracker::start() { impl->start(); }

EXPORT void slam_tracker::stop() { impl->stop(); }

EXPORT void slam_tracker::finalize() { impl->finalize(); }

EXPORT bool slam_tracker::is_running() { return impl->is_running(); }

EXPORT void slam_tracker::push_imu_sample(const imu_sample &s) { impl->push_imu_sample(s); }

EXPORT void slam_tracker::push_frame(const img_sample &sample) { impl->push_frame(sample); }

EXPORT bool slam_tracker::try_dequeue_pose(pose &pose) { return impl->try_dequeue_pose(pose); }

EXPORT bool slam_tracker::supports_feature(int feature_id) { return impl->supports_feature(feature_id); }

EXPORT bool slam_tracker::use_feature(int feature_id, const shared_ptr<void> &params, shared_ptr<void> &result) {
  return impl->use_feature(feature_id, params, result);
}

}  // namespace xrt::auxiliary::tracking::slam
