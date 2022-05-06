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

namespace xrt::auxiliary::tracking::slam {

const int IMPLEMENTATION_VERSION_MAJOR = HEADER_VERSION_MAJOR;
const int IMPLEMENTATION_VERSION_MINOR = HEADER_VERSION_MINOR;
const int IMPLEMENTATION_VERSION_PATCH = HEADER_VERSION_PATCH;

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

string imu2str(const imu_sample &s) {
  string str = "imu_sample ";
  str += "t=" + to_string(s.timestamp) + " ";
  str += "a=[" + to_string(s.ax) + ", " + to_string(s.ay) + ", " + to_string(s.az) + "] ";
  str += "w=[" + to_string(s.wx) + ", " + to_string(s.wy) + ", " + to_string(s.wz) + "]";
  return str;
}

string img2str(const img_sample &s) {
  string str = "img_sample ";
  str += s.is_left ? "left " : "right ";
  str += "t=" + to_string(s.timestamp);
  return str;
}

string pose2str(const pose &p) {
  string str = "pose ";
  str += "p=[" + to_string(p.px) + ", " + to_string(p.py) + ", " + to_string(p.pz) + "] ";
  str += "r=[" + to_string(p.rx) + ", " + to_string(p.ry) + ", " + to_string(p.rz) + ", " + to_string(p.rw) + "]";
  return str;
}

struct slam_tracker::implementation {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  // Options parsed from unified config file
  bool show_gui = true;
  string cam_calib_path;
  string config_path;
  string marg_data_path;
  bool print_queue = false;
  bool use_double = false;

  // Basalt in its current state does not support monocular cameras, although it
  // should be possible to adapt it to do so, see:
  // https://gitlab.com/VladyslavUsenko/basalt/-/issues/2#note_201965760
  // https://gitlab.com/VladyslavUsenko/basalt/-/issues/25#note_362741510
  // https://github.com/DLR-RM/granite
  static constexpr int NUM_CAMS = 2;

  // VIO members
  Calibration<double> calib;
  VioConfig vio_config;
  OpticalFlowBase::Ptr opt_flow_ptr;
  VioEstimatorBase::Ptr vio;
  bool expecting_left_frame = true;

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
  unordered_set<int> supported_features{F_ADD_CAMERA_CALIBRATION, F_ADD_IMU_CALIBRATION, F_ENABLE_POSE_EXT_TIMING};

  // Additional calibration data
  vector<cam_calibration> added_cam_calibs{};
  vector<imu_calibration> added_imu_calibs{};

  // Pose timing
  bool pose_timing_enabled = false;

 public:
  implementation(const string &unified_config) {
    load_unified_config(unified_config);

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
      cout << "Loaded camera with " << calib.intrinsics.size() << " cameras\n";
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

    if (pose_timing_enabled) {
      auto pose_timing = make_shared<pose_ext_timing>();
      pose_timing->timestamps = state->input_images->tss;
      p.next = pose_timing;
    }

    return p;
  }

  void state_consumer() {
    PoseVelBiasState<double>::Ptr data;

    while (true) {
      out_state_queue.pop(data);
      if (data.get() == nullptr) {
        monado_out_state_queue.push(nullptr);
        break;
      }
      data->input_images->addTime("tracker_consumer_received");

      if (show_gui) ui.log_vio_data(data);

      data->input_images->addTime("tracker_consumer_pushed");
      monado_out_state_queue.push(data);
    }

    cout << "Finished state_consumer\n";
  }

  void queues_printer() {
    while (running) {
      cout << "[in] frames: " << image_data_queue->size() << "/" << image_data_queue->capacity() << " \t"
           << "[in] imu: " << imu_data_queue->size() << "/" << imu_data_queue->capacity() << " \t"
           << "[mid] keypoints: " << opt_flow_ptr->output_queue->size() << "/" << opt_flow_ptr->output_queue->capacity()
           << " \t"
           << "[out] pose: " << out_state_queue.size() << "/" << out_state_queue.capacity() << "\n";
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    cout << "Finished queues_printer\n";
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

    // NOTE: This factory also starts the optical flow
    opt_flow_ptr = OpticalFlowFactory::getOpticalFlow(vio_config, calib);
    image_data_queue = &opt_flow_ptr->input_queue;
    ASSERT_(image_data_queue != nullptr);

    vio = VioEstimatorFactory::getVioEstimator(vio_config, calib, constants::g, true, use_double);
    imu_data_queue = &vio->imu_data_queue;
    ASSERT_(imu_data_queue != nullptr);

    opt_flow_ptr->output_queue = &vio->vision_data_queue;
    if (show_gui) {
      ui.initialize(NUM_CAMS);
      vio->out_vis_queue = &ui.out_vis_queue;
    };
    vio->out_state_queue = &out_state_queue;

    if (!marg_data_path.empty()) {
      marg_data_saver.reset(new MargDataSaver(marg_data_path));
      vio->out_marg_queue = &marg_data_saver->in_marg_queue;
    }
  }

  void start() {
    running = true;
    vio->initialize(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    if (show_gui) ui.start(vio->getT_w_i_init(), calib);
    state_consumer_thread = thread(&slam_tracker::implementation::state_consumer, this);
    if (print_queue) queues_printer_thread = thread(&slam_tracker::implementation::queues_printer, this);
  }

  void stop() {
    running = false;
    image_data_queue->push(nullptr);
    imu_data_queue->push(nullptr);

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
  }

 private:
  OpticalFlowInput::Ptr partial_frame;

 public:
  void push_frame(const img_sample &s) {
    ASSERT(expecting_left_frame == s.is_left, "Unexpected %s frame", s.is_left ? "left" : "right");
    expecting_left_frame = !expecting_left_frame;

    int i = -1;
    if (s.is_left) {
      partial_frame = make_shared<OpticalFlowInput>();
      partial_frame->timing_enabled = pose_timing_enabled;
      partial_frame->addTime("frame_ts", s.timestamp);
      partial_frame->addTime("tracker_received");
      partial_frame->img_data.resize(NUM_CAMS);
      partial_frame->t_ns = s.timestamp;
      i = 0;
    } else {
      ASSERT(partial_frame->t_ns == s.timestamp, "Left and right frame timestamps differ: %ld != %ld",
             partial_frame->t_ns, s.timestamp);
      i = 1;
    }

    int width = s.img.cols;
    int height = s.img.rows;
    // Forced to use uint16_t here, in place because of cameras with 12-bit grayscale support
    auto &mimg = partial_frame->img_data[i].img;
    mimg.reset(new ManagedImage<uint16_t>(width, height));

    // TODO: We could avoid this copy. Maybe by writing a custom
    // allocator for ManagedImage that ties the OpenCV allocator
    size_t full_size = width * height;
    for (size_t j = 0; j < full_size; j++) {
      mimg->ptr[j] = s.img.at<uchar>(j) << 8;
    }

    if (!s.is_left) {
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
      result = enable_pose_ext_timing();
    } else {
      return false;
    }
    return true;
  }

  void add_cam_calibration(const cam_calibration &cam_calib) { added_cam_calibs.push_back(cam_calib); }

  void apply_cam_calibration(const cam_calibration &cam_calib) {
    using Scalar = double;
    int i = cam_calib.cam_index;

    const auto &tci = cam_calib.T_cam_imu;
    Eigen::Matrix3d rci;
    rci << tci(0, 0), tci(0, 1), tci(0, 2), tci(1, 0), tci(1, 1), tci(1, 2), tci(2, 0), tci(2, 1), tci(2, 2);
    Eigen::Quaterniond q(rci);
    Eigen::Vector3d p{tci(0, 3), tci(1, 3), tci(2, 3)};
    calib.T_i_c[i] = Calibration<Scalar>::SE3(q, p);

    GenericCamera<double> model;
    const vector<Scalar> &cmp = cam_calib.model_params;
    if (cam_calib.model == cam_calibration::cam_model::pinhole) {
      PinholeCamera<Scalar>::VecN mp;
      mp << cam_calib.fx, cam_calib.fy, cam_calib.cx, cam_calib.cy;
      PinholeCamera pinhole(mp);
      model.variant = pinhole;
    } else if (cam_calib.model == cam_calibration::cam_model::fisheye) {
      KannalaBrandtCamera4<Scalar>::VecN mp;
      mp << cam_calib.fx, cam_calib.fy, cam_calib.cx, cam_calib.cy, cmp[0], cmp[1], cmp[2], cmp[3];
      KannalaBrandtCamera4 kannala_brandt(mp);
      model.variant = kannala_brandt;
    } else {
      ASSERT(false, "Unsupported camera model (%d)", static_cast<int>(cam_calib.model));
    }
    calib.intrinsics[i] = model;

    calib.resolution[i] = {cam_calib.width, cam_calib.height};

    // NOTE: ignoring cam_calib.distortion_model and distortion_params as basalt can't use them
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
  }

  shared_ptr<vector<string>> enable_pose_ext_timing() {
    pose_timing_enabled = true;
    vector<string> titles{"frame_ts",
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
                          "monado_dequeued"};
    return make_shared<vector<string>>(titles);
  }
};

slam_tracker::slam_tracker(const string &config_file) { impl = make_unique<slam_tracker::implementation>(config_file); }

slam_tracker::~slam_tracker() = default;

void slam_tracker::initialize() { impl->initialize(); }

void slam_tracker::start() { impl->start(); }

void slam_tracker::stop() { impl->stop(); }

void slam_tracker::finalize() { impl->finalize(); }

bool slam_tracker::is_running() { return impl->is_running(); }

void slam_tracker::push_imu_sample(const imu_sample &s) { impl->push_imu_sample(s); }

void slam_tracker::push_frame(const img_sample &sample) { impl->push_frame(sample); }

bool slam_tracker::try_dequeue_pose(pose &pose) { return impl->try_dequeue_pose(pose); }

bool slam_tracker::supports_feature(int feature_id) { return impl->supports_feature(feature_id); }

bool slam_tracker::use_feature(int feature_id, const shared_ptr<void> &params, shared_ptr<void> &result) {
  return impl->use_feature(feature_id, params, result);
}

}  // namespace xrt::auxiliary::tracking::slam
