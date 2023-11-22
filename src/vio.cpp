/**
BSD 3-Clause License

This file is part of the Basalt project.
https://gitlab.com/VladyslavUsenko/basalt.git

Copyright (c) 2019, Vladyslav Usenko and Nikolaus Demmel.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <iostream>
#include <thread>

#include <fmt/format.h>

#include <sophus/se3.hpp>

#include <tbb/concurrent_unordered_map.h>
#include <tbb/global_control.h>

#include <pangolin/display/image_view.h>
#include <pangolin/gl/gldraw.h>
#include <pangolin/image/image.h>
#include <pangolin/image/image_io.h>
#include <pangolin/image/typed_image.h>
#include <pangolin/pangolin.h>

#include <CLI/CLI.hpp>

#include <basalt/io/dataset_io.h>
#include <basalt/io/marg_data_io.h>
#include <basalt/spline/se3_spline.h>
#include <basalt/utils/assert.h>
#include <basalt/vi_estimator/vio_estimator.h>
#include <basalt/calibration/calibration.hpp>

#include <basalt/serialization/headers_serialization.h>

#include <basalt/utils/keypoints.h>
#include <basalt/utils/system_utils.h>
#include <basalt/utils/vio_config.h>
#include <basalt/utils/vis_utils.h>
#include <basalt/linearization/landmark_block.hpp>
#include <basalt/utils/format.hpp>
#include <basalt/utils/time_utils.hpp>

// enable the "..."_format(...) string literal
using namespace basalt::literals;
using namespace basalt;
using namespace Eigen;

// GUI functions
void draw_image_overlay(pangolin::ImageView& v, size_t cam_id);
void draw_scene(pangolin::View& view);
void load_data(const std::string& calib_path);
bool next_step();
bool prev_step();
bool toggle_blocks();
bool highlight_frame();
void draw_plots();
void alignButton();
void alignDeviceButton();
void saveTrajectoryButton();

// Pangolin variables
constexpr int UI_WIDTH_PIX = 200;
const pangolin::Attach UI_WIDTH = pangolin::Attach::Pix(UI_WIDTH_PIX);

using Button = pangolin::Var<std::function<void(void)>>;

pangolin::DataLog imu_data_log, vio_data_log, error_data_log;
std::shared_ptr<pangolin::Plotter> plotter;

pangolin::Var<int> show_frame("ui.show_frame", 0, 0, 1500);

pangolin::Var<bool> show_flow("ui.show_flow", false, false, true);
pangolin::Var<bool> show_responses("ui.show_responses", false, false, true);
pangolin::Var<bool> show_tracking_guess("ui.show_tracking_guess", false, false, true);
pangolin::Var<bool> show_matching_guess("ui.show_matching_guess", false, false, true);
pangolin::Var<bool> show_recall_guess("ui.show_recall_guess", false, false, true);
pangolin::Var<bool> show_obs("ui.show_obs", true, false, true);
pangolin::Var<bool> show_ids("ui.show_ids", false, false, true);
pangolin::Var<bool> show_depth{"ui.show_depth", false, false, true};

pangolin::Var<std::string> highlight_landmarks("ui.Highlight", "0, 1, 10-5000");
pangolin::Var<bool> filter_highlights("ui.filter_highlights", false, false, true);
pangolin::Var<bool> show_highlights("ui.show_highlights", false, false, true);
pangolin::Var<bool> follow_highlight("ui.follow_highlight", false, false, true);
Button highlight_frame_btn("ui.highlight_frame", &highlight_frame);

Button toggle_blocks_btn("ui.toggle_blocks", &toggle_blocks);
pangolin::Var<bool> show_block_vals("ui.show_block_vals", false, false, true);

pangolin::Var<bool> show_grid{"ui.show_grid", false, false, true};
pangolin::Var<bool> show_safe_radius{"ui.show_safe_radius", false, false, true};
pangolin::Var<bool> show_cam0_proj{"ui.show_cam0_proj", false, false, true};
pangolin::Var<bool> show_masks{"ui.show_masks", false, false, true};

pangolin::Var<bool> show_guesses{"ui.Show matching guesses", false, false, true};
pangolin::Var<bool> show_same_pixel_guess{"ui.SAME_PIXEL", true, false, true};
pangolin::Var<bool> show_reproj_avg_depth_guess{"ui.REPROJ_AVG_DEPTH", true, false, true};
pangolin::Var<bool> show_reproj_fix_depth_guess{"ui.REPROJ_FIX_DEPTH", true, false, true};
pangolin::Var<double> fixed_depth{"ui.FIX_DEPTH", 2, 0, 3};
pangolin::Var<bool> show_active_guess{"ui.Active Guess", true, false, true};

pangolin::Var<double> depth_guess{"ui.depth_guess", 2, pangolin::META_FLAG_READONLY};

pangolin::Var<bool> show_est_pos("ui.show_est_pos", true, false, true);
pangolin::Var<bool> show_est_vel("ui.show_est_vel", false, false, true);
pangolin::Var<bool> show_est_bg("ui.show_est_bg", true, false, true);
pangolin::Var<bool> show_est_ba("ui.show_est_ba", true, false, true);

pangolin::Var<bool> show_gt("ui.show_gt", true, false, true);

Button next_step_btn("ui.next_step", &next_step);
Button prev_step_btn("ui.prev_step", &prev_step);

pangolin::Var<bool> continue_btn("ui.continue", false, false, true);
pangolin::Var<bool> continue_fast("ui.continue_fast", true, false, true);

Button align_se3_btn("ui.align_se3", &alignButton);

pangolin::Var<bool> euroc_fmt("ui.euroc_fmt", true, false, true);
pangolin::Var<bool> tum_rgbd_fmt("ui.tum_rgbd_fmt", false, false, true);
pangolin::Var<bool> kitti_fmt("ui.kitti_fmt", false, false, true);
pangolin::Var<bool> save_groundtruth("ui.save_groundtruth", false, false, true);
Button save_traj_btn("ui.save_traj", &saveTrajectoryButton);

pangolin::Var<bool> follow("ui.follow", false, false, true);

// pangolin::Var<bool> record("ui.record", false, false, true);

pangolin::OpenGlRenderState camera;

// Visualization variables
std::unordered_map<int64_t, basalt::VioVisualizationData::Ptr> vis_map;

tbb::concurrent_bounded_queue<basalt::VioVisualizationData::Ptr> out_vis_queue;
tbb::concurrent_bounded_queue<basalt::PoseVelBiasState<double>::Ptr> out_state_queue;

std::vector<int64_t> vio_t_ns;
Eigen::aligned_vector<Eigen::Vector3d> vio_t_w_i;
Eigen::aligned_vector<Sophus::SE3d> vio_T_w_i;

std::vector<int64_t> gt_t_ns;
Eigen::aligned_vector<Eigen::Vector3d> gt_t_w_i;

std::string marg_data_path;
size_t last_frame_processed = 0;

tbb::concurrent_unordered_map<int64_t, int, std::hash<int64_t>> timestamp_to_id;

std::mutex m;
std::condition_variable cvar;
bool step_by_step = false;
size_t max_frames = 0;

std::atomic<bool> terminate = false;

// VIO variables
basalt::Calibration<double> calib;

basalt::VioDatasetPtr vio_dataset;
basalt::VioConfig vio_config;
basalt::OpticalFlowBase::Ptr opt_flow_ptr;
basalt::VioEstimatorBase::Ptr vio;
pangolin::View* img_view_display;
pangolin::View* plot_display;
pangolin::View* blocks_display;
std::vector<std::shared_ptr<pangolin::ImageView>> img_view;
bool show_blocks = false;
vis::Selection highlights{};

bool is_highlighted(size_t lmid) { return vis::is_selected(highlights, lmid); }

// Feed functions
void feed_images() {
  std::cout << "Started input_data thread " << std::endl;

  int NUM_CAMS = calib.intrinsics.size();
  for (size_t i = 0; i < vio_dataset->get_image_timestamps().size(); i++) {
    if (vio->finished || terminate || (max_frames > 0 && i >= max_frames)) {
      // stop loop early if we set a limit on number of frames to process
      break;
    }

    if (step_by_step) {
      std::unique_lock<std::mutex> lk(m);
      cvar.wait(lk);
    }

    basalt::OpticalFlowInput::Ptr img(new basalt::OpticalFlowInput(NUM_CAMS));

    img->t_ns = vio_dataset->get_image_timestamps()[i];
    img->img_data = vio_dataset->get_image_data(img->t_ns);

    timestamp_to_id[img->t_ns] = i;

    opt_flow_ptr->input_img_queue.push(img);
  }

  // Indicate the end of the sequence
  opt_flow_ptr->input_img_queue.push(nullptr);

  std::cout << "Finished input_data thread " << std::endl;
}

void feed_imu() {
  for (size_t i = 0; i < vio_dataset->get_gyro_data().size(); i++) {
    if (vio->finished || terminate) {
      break;
    }

    basalt::ImuData<double>::Ptr data(new basalt::ImuData<double>);
    data->t_ns = vio_dataset->get_gyro_data()[i].timestamp_ns;

    data->accel = vio_dataset->get_accel_data()[i].data;
    data->gyro = vio_dataset->get_gyro_data()[i].data;

    vio->imu_data_queue.push(data);
    opt_flow_ptr->input_imu_queue.push(data);
  }
  vio->imu_data_queue.push(nullptr);
  opt_flow_ptr->input_imu_queue.push(nullptr);
}

int main(int argc, char** argv) {
  bool show_gui = true;
  bool print_queue = false;
  std::string cam_calib_path;
  std::string dataset_path;
  std::string dataset_type;
  std::string config_path;
  std::string result_path;
  std::string trajectory_fmt;
  bool trajectory_groundtruth;
  int num_threads = 0;
  bool use_imu = true;
  bool use_double = false;

  CLI::App app{"App description"};

  app.add_option("--show-gui", show_gui, "Show GUI");
  app.add_option("--cam-calib", cam_calib_path, "Ground-truth camera calibration used for simulation.")->required();

  app.add_option("--dataset-path", dataset_path, "Path to dataset.")->required();

  app.add_option("--dataset-type", dataset_type, "Dataset type <euroc, bag>.")->required();

  app.add_option("--marg-data", marg_data_path, "Path to folder where marginalization data will be stored.");

  app.add_option("--print-queue", print_queue, "Print queue.");
  app.add_option("--config-path", config_path, "Path to config file.");
  app.add_option("--result-path", result_path, "Path to result file where the system will write RMSE ATE.");
  app.add_option("--num-threads", num_threads, "Number of threads.");
  app.add_option("--step-by-step", step_by_step, "Path to config file.");
  app.add_option("--save-trajectory", trajectory_fmt, "Save trajectory. Supported formats <tum, euroc, kitti>");
  app.add_option("--save-groundtruth", trajectory_groundtruth, "In addition to trajectory, save also ground turth");
  app.add_option("--use-imu", use_imu, "Use IMU.");
  app.add_option("--use-double", use_double, "Use double not float.");
  app.add_option("--max-frames", max_frames, "Limit number of frames to process from dataset (0 means unlimited)");

  try {
    app.parse(argc, argv);
  } catch (const CLI::ParseError& e) {
    return app.exit(e);
  }

  // global thread limit is in effect until global_control object is destroyed
  std::unique_ptr<tbb::global_control> tbb_global_control;
  if (num_threads > 0) {
    tbb_global_control =
        std::make_unique<tbb::global_control>(tbb::global_control::max_allowed_parallelism, num_threads);
  }

  if (!config_path.empty()) {
    vio_config.load(config_path);

    if (vio_config.vio_enforce_realtime) {
      vio_config.vio_enforce_realtime = false;
      std::cout << "The option vio_config.vio_enforce_realtime was enabled, "
                   "but it should only be used with the live executables (supply "
                   "images at a constant framerate). This executable runs on the "
                   "datasets and processes images as fast as it can, so the option "
                   "will be disabled. "
                << std::endl;
    }
  }

  load_data(cam_calib_path);

  {
    basalt::DatasetIoInterfacePtr dataset_io = basalt::DatasetIoFactory::getDatasetIo(dataset_type);

    dataset_io->read(dataset_path);

    vio_dataset = dataset_io->get_data();

    show_frame.Meta().range[1] = vio_dataset->get_image_timestamps().size() - 1;
    show_frame.Meta().gui_changed = true;

    opt_flow_ptr = basalt::OpticalFlowFactory::getOpticalFlow(vio_config, calib);
    opt_flow_ptr->start();

    for (size_t i = 0; i < vio_dataset->get_gt_pose_data().size(); i++) {
      gt_t_ns.push_back(vio_dataset->get_gt_timestamps()[i]);
      gt_t_w_i.push_back(vio_dataset->get_gt_pose_data()[i].translation());
    }
  }

  const int64_t start_t_ns = vio_dataset->get_image_timestamps().front();
  {
    vio = basalt::VioEstimatorFactory::getVioEstimator(vio_config, calib, basalt::constants::g, use_imu, use_double);
    vio->initialize(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    opt_flow_ptr->output_queue = &vio->vision_data_queue;
    opt_flow_ptr->show_gui = show_gui;
    if (show_gui) vio->out_vis_queue = &out_vis_queue;
    vio->out_state_queue = &out_state_queue;
    vio->opt_flow_depth_guess_queue = &opt_flow_ptr->input_depth_queue;
    vio->opt_flow_state_queue = &opt_flow_ptr->input_state_queue;
    vio->opt_flow_lm_bundle_queue = &opt_flow_ptr->input_lm_bundle_queue;
  }

  basalt::MargDataSaver::Ptr marg_data_saver;

  if (!marg_data_path.empty()) {
    marg_data_saver.reset(new basalt::MargDataSaver(marg_data_path));
    vio->out_marg_queue = &marg_data_saver->in_marg_queue;

    // Save gt.
    {
      std::string p = marg_data_path + "/gt.cereal";
      std::ofstream os(p, std::ios::binary);

      {
        cereal::BinaryOutputArchive archive(os);
        archive(gt_t_ns);
        archive(gt_t_w_i);
      }
      os.close();
    }
  }

  vio_data_log.Clear();

  std::thread t1(&feed_images);
  std::thread t2(&feed_imu);

  std::shared_ptr<std::thread> t3;

  if (show_gui)
    t3.reset(new std::thread([&]() {
      basalt::VioVisualizationData::Ptr data;

      while (true) {
        out_vis_queue.pop(data);

        if (data.get()) {
          vis_map[data->t_ns] = data;
        } else {
          break;
        }
      }

      std::cout << "Finished t3" << std::endl;
    }));

  std::thread t4([&]() {
    basalt::PoseVelBiasState<double>::Ptr data;

    while (true) {
      out_state_queue.pop(data);

      if (!data.get()) break;

      int64_t t_ns = data->t_ns;

      // std::cerr << "t_ns " << t_ns << std::endl;
      Sophus::SE3d T_w_i = data->T_w_i;
      Eigen::Vector3d vel_w_i = data->vel_w_i;
      Eigen::Vector3d bg = data->bias_gyro;
      Eigen::Vector3d ba = data->bias_accel;

      vio_t_ns.emplace_back(data->t_ns);
      vio_t_w_i.emplace_back(T_w_i.translation());
      vio_T_w_i.emplace_back(T_w_i);

      if (show_gui) {
        std::vector<float> vals;
        vals.push_back((t_ns - start_t_ns) * 1e-9);

        for (int i = 0; i < 3; i++) vals.push_back(vel_w_i[i]);
        for (int i = 0; i < 3; i++) vals.push_back(T_w_i.translation()[i]);
        for (int i = 0; i < 3; i++) vals.push_back(bg[i]);
        for (int i = 0; i < 3; i++) vals.push_back(ba[i]);

        vio_data_log.Log(vals);
      }
    }

    std::cout << "Finished t4" << std::endl;
  });

  std::shared_ptr<std::thread> t5;

  auto print_queue_fn = [&]() {
    std::cout << "opt_flow_ptr->input_img_queue " << opt_flow_ptr->input_img_queue.size()
              << " opt_flow_ptr->output_queue " << opt_flow_ptr->output_queue->size() << " out_state_queue "
              << out_state_queue.size() << " imu_data_queue " << vio->imu_data_queue.size() << std::endl;
  };

  if (print_queue) {
    t5.reset(new std::thread([&]() {
      while (!terminate) {
        print_queue_fn();
        std::this_thread::sleep_for(std::chrono::seconds(1));
      }
    }));
  }

  auto time_start = std::chrono::high_resolution_clock::now();

  // record if we close the GUI before VIO is finished.
  bool aborted = false;

  if (show_gui) {
    pangolin::CreateWindowAndBind("Main", 1800, 1000);

    glEnable(GL_DEPTH_TEST);

    pangolin::View& main_display = pangolin::CreateDisplay();
    main_display.SetBounds(0.0, 1.0, UI_WIDTH, 1.0);

    img_view_display = &pangolin::CreateDisplay();
    img_view_display->SetBounds(0.4, 1.0, 0.0, 0.4);
    img_view_display->SetLayout(pangolin::LayoutEqual);

    plotter = std::make_shared<pangolin::Plotter>(&imu_data_log, 0.0, 100, -10.0, 10.0, 0.01, 0.01);
    plot_display = &pangolin::CreateDisplay();
    plot_display->SetBounds(0.0, 0.4, UI_WIDTH, 1.0);
    plot_display->AddDisplay(*plotter);

    auto blocks_view = std::make_shared<pangolin::ImageView>();
    blocks_view->UseNN() = true;  // Disable antialiasing, can be toggled with N key
    blocks_view->extern_draw_function = [](pangolin::View& v) {
      vis::draw_blocks_overlay_vio(show_frame, vio_dataset, vis_map, dynamic_cast<pangolin::ImageView&>(v), highlights,
                                   filter_highlights, show_highlights, show_block_vals, show_ids);
    };
    const int DEFAULT_W = 480;
    blocks_display = &pangolin::CreateDisplay();
    blocks_display->SetBounds(0.0, 0.6, UI_WIDTH, pangolin::Attach::Pix(UI_WIDTH_PIX + DEFAULT_W));
    blocks_display->AddDisplay(*blocks_view);
    blocks_display->Show(show_blocks);

    pangolin::CreatePanel("ui").SetBounds(0.0, 1.0, 0.0, UI_WIDTH);

    while (img_view.size() < calib.intrinsics.size()) {
      std::shared_ptr<pangolin::ImageView> iv(new pangolin::ImageView);
      iv->UseNN() = true;  // Disable antialiasing, can be toggled with N key

      size_t idx = img_view.size();
      img_view.push_back(iv);

      img_view_display->AddDisplay(*iv);
      iv->extern_draw_function = [idx](pangolin::View& v) {
        return draw_image_overlay(dynamic_cast<pangolin::ImageView&>(v), idx);
      };
    }

    Eigen::Vector3d cam_p(-0.5, -3, -5);
    cam_p = vio->getT_w_i_init().so3() * calib.T_i_c[0].so3() * cam_p;

    camera =
        pangolin::OpenGlRenderState(pangolin::ProjectionMatrix(640, 480, 400, 400, 320, 240, 0.001, 10000),
                                    pangolin::ModelViewLookAt(cam_p[0], cam_p[1], cam_p[2], 0, 0, 0, pangolin::AxisZ));

    pangolin::View& display3D = pangolin::CreateDisplay();
    display3D.SetAspect(-640 / 480.0);
    display3D.SetBounds(0.4, 1.0, 0.4, 1.0);
    display3D.SetHandler(new pangolin::Handler3D(camera));

    display3D.extern_draw_function = draw_scene;

    main_display.AddDisplay(*img_view_display);
    main_display.AddDisplay(display3D);

    while (!pangolin::ShouldQuit()) {
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

      size_t frame_id = show_frame;
      int64_t t_ns = vio_dataset->get_image_timestamps()[frame_id];
      auto it = vis_map.find(t_ns);
      if (follow) {
        if (it != vis_map.end()) {
          Sophus::SE3d T_w_i;
          if (!it->second->states.empty()) {
            T_w_i = it->second->states.back();
          } else if (!it->second->frames.empty()) {
            T_w_i = it->second->frames.back();
          }
          T_w_i.so3() = Sophus::SO3d();

          camera.Follow(T_w_i.matrix());
        }
      }

      display3D.Activate(camera);
      glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

      img_view_display->Activate();
      if (fixed_depth.GuiChanged() && vio->opt_flow_depth_guess_queue != nullptr) {
        vio->opt_flow_depth_guess_queue->push(fixed_depth);
        depth_guess = fixed_depth;
      } else if (it != vis_map.end() && it->second->opt_flow_res && it->second->opt_flow_res->input_images) {
        depth_guess = it->second->opt_flow_res->input_images->depth_guess;
      }

      if (show_frame.GuiChanged()) {
        for (size_t cam_id = 0; cam_id < calib.intrinsics.size(); cam_id++) {
          size_t frame_id = static_cast<size_t>(show_frame);
          int64_t timestamp = vio_dataset->get_image_timestamps()[frame_id];

          std::vector<basalt::ImageData> img_vec = vio_dataset->get_image_data(timestamp);

          pangolin::GlPixFormat fmt;
          fmt.glformat = GL_LUMINANCE;
          fmt.gltype = GL_UNSIGNED_SHORT;
          fmt.scalable_internal_format = GL_LUMINANCE16;

          if (img_vec[cam_id].img.get())
            img_view[cam_id]->SetImage(img_vec[cam_id].img->ptr, img_vec[cam_id].img->w, img_vec[cam_id].img->h,
                                       img_vec[cam_id].img->pitch, fmt);
        }
        if (follow_highlight) vis::follow_highlight_vio(frame_id, vio_dataset, vis_map, img_view, highlights, false);

        if (show_blocks && it != vis_map.end())
          vis::show_blocks(it->second, blocks_view, highlights, filter_highlights);

        draw_plots();
      }

      if (show_est_vel.GuiChanged() || show_est_pos.GuiChanged() || show_est_ba.GuiChanged() ||
          show_est_bg.GuiChanged()) {
        draw_plots();
      }

      if (highlight_landmarks.GuiChanged() || filter_highlights.GuiChanged() || show_highlights.GuiChanged()) {
        highlights = vis::parse_selection(highlight_landmarks);
        filter_highlights = filter_highlights && !highlights.empty();
        for (const auto& [ts, vis] : vis_map) vis->mat = nullptr;  // Regenerate lmbs
        if (show_blocks && it != vis_map.end())
          vis::show_blocks(it->second, blocks_view, highlights, filter_highlights);
      }

      if (follow_highlight.GuiChanged()) {
        follow_highlight = follow_highlight && highlights.size() == 1 && !highlights[0].is_range;
        if (follow_highlight)
          vis::follow_highlight_vio(frame_id, vio_dataset, vis_map, img_view, highlights, true);
        else
          for (auto& v : img_view) v->ResetView();
      }

      if (euroc_fmt.GuiChanged()) {
        euroc_fmt = true;
        tum_rgbd_fmt = false;
        kitti_fmt = false;
      }

      if (tum_rgbd_fmt.GuiChanged()) {
        tum_rgbd_fmt = true;
        euroc_fmt = false;
        kitti_fmt = false;
      }

      if (kitti_fmt.GuiChanged()) {
        kitti_fmt = true;
        euroc_fmt = false;
        tum_rgbd_fmt = false;
      }

      //      if (record) {
      //        main_display.RecordOnRender(
      //            "ffmpeg:[fps=50,bps=80000000,unique_filename]///tmp/"
      //            "vio_screencap.avi");
      //        record = false;
      //      }

      pangolin::FinishFrame();

      if (continue_btn) {
        if (!next_step()) std::this_thread::sleep_for(std::chrono::milliseconds(50));
      } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }

      if (continue_fast) {
        int64_t t_ns = vio->last_processed_t_ns;
        if (timestamp_to_id.count(t_ns)) {
          show_frame = timestamp_to_id[t_ns];
          show_frame.Meta().gui_changed = true;
        }

        if (vio->finished) {
          continue_fast = false;
        }
      }
    }

    // If GUI closed but VIO not yet finished --> abort input queues, which in
    // turn aborts processing
    if (!vio->finished) {
      std::cout << "GUI closed but odometry still running --> aborting.\n";
      print_queue_fn();  // print queue size at time of aborting
      terminate = true;
      aborted = true;
    }
  }

  // wait first for vio to complete processing
  vio->maybe_join();

  // input threads will abort when vio is finished, but might be stuck in full
  // push to full queue, so drain queue now
  vio->drain_input_queues();
  opt_flow_ptr->drain_input_queues();

  // join input threads
  t1.join();
  t2.join();

  // std::cout << "Data input finished, terminate auxiliary threads.";
  terminate = true;

  // join other threads
  if (t3) t3->join();
  t4.join();
  if (t5) t5->join();

  // after joining all threads, print final queue sizes.
  if (print_queue) {
    std::cout << "Final queue sizes:" << std::endl;
    print_queue_fn();
  }

  auto time_end = std::chrono::high_resolution_clock::now();
  const double duration_total = std::chrono::duration<double>(time_end - time_start).count();

  // TODO: remove this unconditional call (here for debugging);
  const double ate_rmse = basalt::alignSVD(vio_t_ns, vio_t_w_i, gt_t_ns, gt_t_w_i);
  vio->debug_finalize();
  std::cout << "Total runtime: {:.3f}s\n"_format(duration_total);

  {
    basalt::ExecutionStats stats;
    stats.add("exec_time_s", duration_total);
    stats.add("ate_rmse", ate_rmse);
    stats.add("ate_num_kfs", vio_t_w_i.size());
    stats.add("num_frames", vio_dataset->get_image_timestamps().size());

    {
      basalt::MemoryInfo mi;
      if (get_memory_info(mi)) {
        stats.add("resident_memory_peak", mi.resident_memory_peak);
      }
    }

    stats.save_json("stats_vio.json");
  }

  if (!aborted && !trajectory_fmt.empty()) {
    std::cout << "Saving trajectory..." << std::endl;

    if (trajectory_fmt == "kitti") {
      kitti_fmt = true;
      euroc_fmt = false;
      tum_rgbd_fmt = false;
    }
    if (trajectory_fmt == "euroc") {
      euroc_fmt = true;
      kitti_fmt = false;
      tum_rgbd_fmt = false;
    }
    if (trajectory_fmt == "tum") {
      tum_rgbd_fmt = true;
      euroc_fmt = false;
      kitti_fmt = false;
    }

    save_groundtruth = trajectory_groundtruth;

    saveTrajectoryButton();
  }

  if (!aborted && !result_path.empty()) {
    double error = basalt::alignSVD(vio_t_ns, vio_t_w_i, gt_t_ns, gt_t_w_i);

    auto exec_time_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(time_end - time_start);

    std::ofstream os(result_path);
    {
      cereal::JSONOutputArchive ar(os);
      ar(cereal::make_nvp("rms_ate", error));
      ar(cereal::make_nvp("num_frames", vio_dataset->get_image_timestamps().size()));
      ar(cereal::make_nvp("exec_time_ns", exec_time_ns.count()));
    }
    os.close();
  }

  return 0;
}

void draw_image_overlay(pangolin::ImageView& v, size_t cam_id) {
  int64_t curr_ts = vio_dataset->get_image_timestamps().at(show_frame);
  auto it = vis_map.find(curr_ts);
  if (it == vis_map.end()) return;
  basalt::VioVisualizationData::Ptr curr_vis_data = it->second;

  if (!curr_vis_data ||                                               //
      !curr_vis_data->opt_flow_res ||                                 //
      !curr_vis_data->opt_flow_res->input_images ||                   //
      curr_vis_data->opt_flow_res->input_images->img_data.empty() ||  //
      !curr_vis_data->opt_flow_res->input_images->img_data.at(0).img) {
    return;
  }

  if (show_obs) {
    vis::show_obs(cam_id, curr_vis_data, v, vio_config, calib, highlights, filter_highlights, show_same_pixel_guess,
                  show_reproj_fix_depth_guess, show_reproj_avg_depth_guess, show_active_guess, fixed_depth, show_ids,
                  show_depth, show_guesses);
  }

  if (show_flow)
    vis::show_flow(cam_id, curr_vis_data, v, opt_flow_ptr, highlights, filter_highlights, show_ids, show_responses);

  if (show_highlights) vis::show_highlights(cam_id, curr_vis_data, highlights, v, show_ids);

  if (show_tracking_guess)
    vis::show_tracking_guess_vio(cam_id, show_frame, vio_dataset, vis_map, highlights, filter_highlights);

  if (show_matching_guess) vis::show_matching_guesses(cam_id, curr_vis_data, highlights, filter_highlights);

  if (show_recall_guess) vis::show_recall_guesses(cam_id, curr_vis_data, highlights, filter_highlights);

  if (show_masks) vis::show_masks(cam_id, curr_vis_data);

  if (show_cam0_proj) vis::show_cam0_proj(cam_id, depth_guess, vio_config, calib);

  if (show_grid) vis::show_grid(vio_config, calib);

  if (show_safe_radius) vis::show_safe_radius(vio_config, calib);
}

void draw_scene(pangolin::View& view) {
  UNUSED(view);
  view.Activate(camera);
  glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

  glPointSize(3);
  glColor3f(1.0, 0.0, 0.0);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  glColor3ubv(cam_color);
  if (!vio_t_w_i.empty()) {
    size_t end = std::min(vio_t_w_i.size(), size_t(show_frame + 1));
    Eigen::aligned_vector<Eigen::Vector3d> sub_gt(vio_t_w_i.begin(), vio_t_w_i.begin() + end);
    pangolin::glDrawLineStrip(sub_gt);
  }

  glColor3ubv(gt_color);
  if (show_gt) pangolin::glDrawLineStrip(gt_t_w_i);

  size_t frame_id = show_frame;
  int64_t t_ns = vio_dataset->get_image_timestamps()[frame_id];
  auto it = vis_map.find(t_ns);
  VioVisualizationData::Ptr curr_vis_data = it->second;

  if (it != vis_map.end()) {
    for (size_t i = 0; i < calib.T_i_c.size(); i++)
      if (!it->second->states.empty()) {
        render_camera((it->second->states.back() * calib.T_i_c[i]).matrix(), 2.0f, cam_color, 0.1f);
      } else if (!it->second->frames.empty()) {
        render_camera((it->second->frames.back() * calib.T_i_c[i]).matrix(), 2.0f, cam_color, 0.1f);
      }

    for (const auto& p : it->second->states)
      for (size_t i = 0; i < calib.T_i_c.size(); i++)
        render_camera((p * calib.T_i_c[i]).matrix(), 2.0f, state_color, 0.1f);

    for (const auto& p : it->second->frames)
      for (size_t i = 0; i < calib.T_i_c.size(); i++)
        render_camera((p * calib.T_i_c[i]).matrix(), 2.0f, pose_color, 0.1f);

    glColor3ubv(pose_color);
    if (!filter_highlights) pangolin::glDrawPoints(it->second->points);

    Eigen::aligned_vector<Eigen::Vector3d> highlighted_points;
    if (show_highlights || filter_highlights) {
      for (size_t i = 0; i < curr_vis_data->point_ids.size(); i++) {
        Vector3d pos = curr_vis_data->points.at(i);
        int id = curr_vis_data->point_ids.at(i);
        if (is_highlighted(id)) highlighted_points.push_back(pos);
      }
    }

    if (filter_highlights) pangolin::glDrawPoints(highlighted_points);

    if (show_highlights) {
      glColor3ubv(vis::GREEN);
      glPointSize(10);
      pangolin::glDrawPoints(highlighted_points);
    }
  }

  glColor3ubv(pose_color);
  if (show_ids) {
    for (size_t i = 0; i < curr_vis_data->points.size(); i++) {
      Vector3d pos = curr_vis_data->points.at(i);
      int id = curr_vis_data->point_ids.at(i);

      bool highlighted = is_highlighted(id);
      if (filter_highlights && !highlighted) continue;

      if (show_highlights && highlighted) glColor3ubv(vis::GREEN);
      pangolin::GlFont::I().Text("%d", id).Draw(pos.x(), pos.y(), pos.z());
      if (show_highlights && highlighted) glColor3ubv(pose_color);
    }
  }

  pangolin::glDrawAxis(Sophus::SE3d().matrix(), 1.0);
}

void load_data(const std::string& calib_path) {
  std::ifstream os(calib_path, std::ios::binary);

  if (os.is_open()) {
    cereal::JSONInputArchive archive(os);
    archive(calib);
    std::cout << "Loaded camera with " << calib.intrinsics.size() << " cameras" << std::endl;

  } else {
    std::cerr << "could not load camera calibration " << calib_path << std::endl;
    std::abort();
  }
}

bool next_step() {
  if (show_frame < int(vio_dataset->get_image_timestamps().size()) - 1) {
    show_frame = show_frame + 1;
    show_frame.Meta().gui_changed = true;
    cvar.notify_one();
    return true;
  } else {
    return false;
  }
}

bool prev_step() {
  if (show_frame >= 1) {
    show_frame = show_frame - 1;
    show_frame.Meta().gui_changed = true;
    return true;
  } else {
    return false;
  }
}

bool toggle_blocks() {
  show_blocks = vis::toggle_blocks(blocks_display, plot_display, img_view_display, UI_WIDTH);
  return show_blocks;
}

bool highlight_frame() {
  int64_t curr_ts = vio_dataset->get_image_timestamps().at(show_frame);
  auto it = vis_map.find(curr_ts);
  if (it == vis_map.end()) return false;
  basalt::VioVisualizationData::Ptr curr_vis_data = it->second;

  std::set<KeypointId> kpids;
  for (const auto& kps : curr_vis_data->opt_flow_res->keypoints)
    for (const auto& [kpid, kp] : kps) kpids.insert(kpid);

  std::string str{};
  str.reserve(kpids.size() * 10);
  for (const KeypointId& kpid : kpids) str += std::to_string(kpid) + ",";
  if (!str.empty()) str.pop_back();

  highlight_landmarks = str;
  highlight_landmarks.Meta().gui_changed = true;

  return true;
}

void draw_plots() {
  plotter->ClearSeries();
  plotter->ClearMarkers();

  if (show_est_pos) {
    plotter->AddSeries("$0", "$4", pangolin::DrawingModeLine, pangolin::Colour::Red(), "position x", &vio_data_log);
    plotter->AddSeries("$0", "$5", pangolin::DrawingModeLine, pangolin::Colour::Green(), "position y", &vio_data_log);
    plotter->AddSeries("$0", "$6", pangolin::DrawingModeLine, pangolin::Colour::Blue(), "position z", &vio_data_log);
  }

  if (show_est_vel) {
    plotter->AddSeries("$0", "$1", pangolin::DrawingModeLine, pangolin::Colour::Red(), "velocity x", &vio_data_log);
    plotter->AddSeries("$0", "$2", pangolin::DrawingModeLine, pangolin::Colour::Green(), "velocity y", &vio_data_log);
    plotter->AddSeries("$0", "$3", pangolin::DrawingModeLine, pangolin::Colour::Blue(), "velocity z", &vio_data_log);
  }

  if (show_est_bg) {
    plotter->AddSeries("$0", "$7", pangolin::DrawingModeLine, pangolin::Colour::Red(), "gyro bias x", &vio_data_log);
    plotter->AddSeries("$0", "$8", pangolin::DrawingModeLine, pangolin::Colour::Green(), "gyro bias y", &vio_data_log);
    plotter->AddSeries("$0", "$9", pangolin::DrawingModeLine, pangolin::Colour::Blue(), "gyro bias z", &vio_data_log);
  }

  if (show_est_ba) {
    plotter->AddSeries("$0", "$10", pangolin::DrawingModeLine, pangolin::Colour::Red(), "accel bias x", &vio_data_log);
    plotter->AddSeries("$0", "$11", pangolin::DrawingModeLine, pangolin::Colour::Green(), "accel bias y",
                       &vio_data_log);
    plotter->AddSeries("$0", "$12", pangolin::DrawingModeLine, pangolin::Colour::Blue(), "accel bias z", &vio_data_log);
  }

  double t = vio_dataset->get_image_timestamps()[show_frame] * 1e-9;
  plotter->AddMarker(pangolin::Marker::Vertical, t, pangolin::Marker::Equal, pangolin::Colour::White());
}

void alignButton() { basalt::alignSVD(vio_t_ns, vio_t_w_i, gt_t_ns, gt_t_w_i); }

void saveTrajectoryButton() {
  if (tum_rgbd_fmt) {
    {
      std::ofstream os("trajectory.txt");

      os << "# timestamp tx ty tz qx qy qz qw" << std::endl;

      for (size_t i = 0; i < vio_t_ns.size(); i++) {
        const Sophus::SE3d& pose = vio_T_w_i[i];
        os << std::scientific << std::setprecision(18) << vio_t_ns[i] * 1e-9 << " " << pose.translation().x() << " "
           << pose.translation().y() << " " << pose.translation().z() << " " << pose.unit_quaternion().x() << " "
           << pose.unit_quaternion().y() << " " << pose.unit_quaternion().z() << " " << pose.unit_quaternion().w()
           << std::endl;
      }

      os.close();
    }

    if (save_groundtruth) {
      std::ofstream os("groundtruth.txt");

      os << "# timestamp tx ty tz qx qy qz qw" << std::endl;

      for (size_t i = 0; i < gt_t_ns.size(); i++) {
        const Eigen::Vector3d& pos = gt_t_w_i[i];
        os << std::scientific << std::setprecision(18) << gt_t_ns[i] * 1e-9 << " " << pos.x() << " " << pos.y() << " "
           << pos.z() << " "
           << "0 0 0 1" << std::endl;
      }

      os.close();
    }

    std::cout << "Saved trajectory in TUM RGB-D Dataset format in trajectory.txt" << std::endl;
  } else if (euroc_fmt) {
    std::ofstream os("trajectory.csv");

    os << "#timestamp [ns],p_RS_R_x [m],p_RS_R_y [m],p_RS_R_z [m],q_RS_w "
          "[],q_RS_x [],q_RS_y [],q_RS_z []"
       << std::endl;

    for (size_t i = 0; i < vio_t_ns.size(); i++) {
      const Sophus::SE3d& pose = vio_T_w_i[i];
      os << std::fixed << std::setprecision(10) << vio_t_ns[i] << "," << pose.translation().x() << ","
         << pose.translation().y() << "," << pose.translation().z() << "," << pose.unit_quaternion().w() << ","
         << pose.unit_quaternion().x() << "," << pose.unit_quaternion().y() << "," << pose.unit_quaternion().z()
         << std::endl;
    }

    std::cout << "Saved trajectory in Euroc Dataset format in trajectory.csv" << std::endl;
  } else {
    std::ofstream os("trajectory_kitti.txt");

    for (size_t i = 0; i < vio_t_ns.size(); i++) {
      Eigen::Matrix<double, 3, 4> mat = vio_T_w_i[i].matrix3x4();
      os << std::scientific << std::setprecision(12) << mat.row(0) << " " << mat.row(1) << " " << mat.row(2) << " "
         << std::endl;
    }

    os.close();

    std::cout << "Saved trajectory in KITTI Dataset format in trajectory_kitti.txt" << std::endl;
  }
}
