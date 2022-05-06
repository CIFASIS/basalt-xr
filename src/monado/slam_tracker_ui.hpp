#pragma once

#include <pangolin/display/image_view.h>
#include <pangolin/pangolin.h>

#include <CLI/CLI.hpp>

#include <cstdio>
#include <memory>
#include <string>
#include <thread>
#include "sophus/se3.hpp"

#include <basalt/io/marg_data_io.h>
#include <basalt/serialization/headers_serialization.h>
#include <basalt/vi_estimator/vio_estimator.h>
#include "basalt/utils/vis_utils.h"

#define ASSERT(cond, ...)                                      \
  do {                                                         \
    if (!(cond)) {                                             \
      printf("Assertion failed @%s:%d\n", __func__, __LINE__); \
      printf(__VA_ARGS__);                                     \
      printf("\n");                                            \
      exit(EXIT_FAILURE);                                      \
    }                                                          \
  } while (false);
#define ASSERT_(cond) ASSERT(cond, "%s", #cond);

namespace xrt::auxiliary::tracking::slam {

using std::cout;
using std::make_shared;
using std::shared_ptr;
using std::string;
using std::thread;
using std::to_string;
using std::vector;
using namespace basalt;

class slam_tracker_ui {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  VioVisualizationData::Ptr curr_vis_data;
  pangolin::DataLog vio_data_log;
  thread vis_thread;
  thread ui_runner_thread;
  size_t num_cams = 0;
  std::atomic<bool> running = false;

 public:
  tbb::concurrent_bounded_queue<VioVisualizationData::Ptr> out_vis_queue{};

  void initialize(int ncams) {
    vio_data_log.Clear();
    ASSERT_(ncams > 0);
    num_cams = ncams;
  }

  void start(const Sophus::SE3d &T_w_i_init, const basalt::Calibration<double> &calib) {
    running = true;
    start_visualization_thread();
    start_ui(T_w_i_init, calib);
  }

  void stop() {
    running = false;
    vis_thread.join();
    ui_runner_thread.join();
  }

  void start_visualization_thread() {
    vis_thread = thread([&]() {
      while (true) {
        out_vis_queue.pop(curr_vis_data);
        if (curr_vis_data.get() == nullptr) break;
      }
      cout << "Finished vis_thread\n";
    });
  }

  int64_t start_t_ns = -1;
  std::vector<int64_t> vio_t_ns;
  Eigen::aligned_vector<Eigen::Vector3d> vio_t_w_i;

  void log_vio_data(const PoseVelBiasState<double>::Ptr &data) {
    int64_t t_ns = data->t_ns;
    if (start_t_ns < 0) start_t_ns = t_ns;
    float since_start_ns = t_ns - start_t_ns;

    vio_t_ns.emplace_back(t_ns);
    vio_t_w_i.emplace_back(data->T_w_i.translation());

    Sophus::SE3d T_w_i = data->T_w_i;
    Eigen::Vector3d vel_w_i = data->vel_w_i;
    Eigen::Vector3d bg = data->bias_gyro;
    Eigen::Vector3d ba = data->bias_accel;

    vector<float> vals;
    vals.push_back(since_start_ns * 1e-9);
    for (int i = 0; i < 3; i++) vals.push_back(vel_w_i[i]);
    for (int i = 0; i < 3; i++) vals.push_back(T_w_i.translation()[i]);
    for (int i = 0; i < 3; i++) vals.push_back(bg[i]);
    for (int i = 0; i < 3; i++) vals.push_back(ba[i]);

    vio_data_log.Log(vals);
  }

  pangolin::Plotter *plotter;
  pangolin::DataLog imu_data_log{};
  basalt::Calibration<double> calib;
  void start_ui(const Sophus::SE3d &T_w_i_init, const basalt::Calibration<double> &c) {
    ui_runner_thread = thread(&slam_tracker_ui::ui_runner, this, T_w_i_init, c);
  }

  pangolin::Var<bool> follow{"ui.follow", true, false, true};
  pangolin::Var<bool> show_est_pos{"ui.show_est_pos", true, false, true};
  pangolin::Var<bool> show_est_vel{"ui.show_est_vel", false, false, true};
  pangolin::Var<bool> show_est_bg{"ui.show_est_bg", false, false, true};
  pangolin::Var<bool> show_est_ba{"ui.show_est_ba", false, false, true};

  void ui_runner(const Sophus::SE3d &T_w_i_init, const basalt::Calibration<double> &c) {
    constexpr int UI_WIDTH = 200;

    calib = c;
    string window_name = "Basalt SLAM Tracker for Monado";
    pangolin::CreateWindowAndBind(window_name, 1800, 1000);

    glEnable(GL_DEPTH_TEST);

    pangolin::View &img_view_display = pangolin::CreateDisplay()
                                           .SetBounds(0.4, 1.0, pangolin::Attach::Pix(UI_WIDTH), 0.4)
                                           .SetLayout(pangolin::LayoutEqual);

    pangolin::View &plot_display = pangolin::CreateDisplay().SetBounds(0.0, 0.4, pangolin::Attach::Pix(UI_WIDTH), 1.0);

    plotter = new pangolin::Plotter(&imu_data_log, 0.0, 100, -3.0, 3.0, 0.01, 0.01);
    plot_display.AddDisplay(*plotter);

    pangolin::CreatePanel("ui").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(UI_WIDTH));

    std::vector<std::shared_ptr<pangolin::ImageView>> img_view;
    while (img_view.size() < calib.intrinsics.size()) {
      std::shared_ptr<pangolin::ImageView> iv(new pangolin::ImageView);

      size_t idx = img_view.size();
      img_view.push_back(iv);

      img_view_display.AddDisplay(*iv);
      iv->extern_draw_function = [idx, this](auto &v) { this->draw_image_overlay(v, idx); };
    }

    Eigen::Vector3d cam_p(0.5, -2, -2);
    cam_p = T_w_i_init.so3() * calib.T_i_c[0].so3() * cam_p;
    cam_p[2] = 1;

    pangolin::OpenGlRenderState camera(
        pangolin::ProjectionMatrix(640, 480, 400, 400, 320, 240, 0.001, 10000),
        pangolin::ModelViewLookAt(cam_p[0], cam_p[1], cam_p[2], 0, 0, 0, pangolin::AxisZ));

    pangolin::View &display3D = pangolin::CreateDisplay()
                                    .SetAspect(-640 / 480.0)
                                    .SetBounds(0.4, 1.0, 0.4, 1.0)
                                    .SetHandler(new pangolin::Handler3D(camera));

    while (running && !pangolin::ShouldQuit()) {
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

      if (follow) {
        // TODO: There is a small race condition here over
        // curr_vis_data that is also present in the original basalt examples
        if (curr_vis_data) {
          auto T_w_i = curr_vis_data->states.back();
          T_w_i.so3() = Sophus::SO3d();

          camera.Follow(T_w_i.matrix());
        }
      }

      display3D.Activate(camera);
      glClearColor(1.0, 1.0, 1.0, 1.0);

      draw_scene();

      img_view_display.Activate();

      {
        pangolin::GlPixFormat fmt;
        fmt.glformat = GL_LUMINANCE;
        fmt.gltype = GL_UNSIGNED_SHORT;
        fmt.scalable_internal_format = GL_LUMINANCE16;

        if (curr_vis_data && curr_vis_data->opt_flow_res && curr_vis_data->opt_flow_res->input_images) {
          auto &img_data = curr_vis_data->opt_flow_res->input_images->img_data;

          for (size_t cam_id = 0; cam_id < num_cams; cam_id++) {
            if (img_data[cam_id].img) {
              img_view[cam_id]->SetImage(img_data[cam_id].img->ptr, img_data[cam_id].img->w, img_data[cam_id].img->h,
                                         img_data[cam_id].img->pitch, fmt);
            }
          }
        }

        draw_plots();
      }

      if (show_est_vel.GuiChanged() || show_est_pos.GuiChanged() || show_est_ba.GuiChanged() ||
          show_est_bg.GuiChanged()) {
        draw_plots();
      }

      pangolin::FinishFrame();
    }

    pangolin::DestroyWindow(window_name);
    cout << "Finished ui_runner\n";
  }

  pangolin::Var<bool> show_obs{"ui.show_obs", true, false, true};
  pangolin::Var<bool> show_ids{"ui.show_ids", false, false, true};

  void draw_image_overlay(pangolin::View &v, size_t cam_id) {
    UNUSED(v);

    if (show_obs) {
      glLineWidth(1.0);
      glColor3f(1.0, 0.0, 0.0);
      glEnable(GL_BLEND);
      glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

      if (curr_vis_data && cam_id < curr_vis_data->projections.size()) {
        const auto &points = curr_vis_data->projections[cam_id];

        if (!points.empty()) {
          double min_id = points[0][2];
          double max_id = points[0][2];

          for (const auto &points2 : curr_vis_data->projections) {
            for (const auto &p : points2) {
              min_id = std::min(min_id, p[2]);
              max_id = std::max(max_id, p[2]);
            }
          }

          for (const auto &c : points) {
            const float radius = 6.5;

            float r;
            float g;
            float b;
            getcolor(c[2] - min_id, max_id - min_id, b, g, r);
            glColor3f(r, g, b);

            pangolin::glDrawCirclePerimeter(c[0], c[1], radius);

            if (show_ids) pangolin::GlFont::I().Text("%d", int(c[3])).Draw(c[0], c[1]);
          }
        }

        glColor3f(1.0, 0.0, 0.0);
        pangolin::GlFont::I().Text("Tracked %d points", points.size()).Draw(5, 20);
      }
    }
  }

  void draw_scene() {
    glPointSize(3);
    glColor3f(1.0, 0.0, 0.0);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glColor3ubv(cam_color);
    Eigen::aligned_vector<Eigen::Vector3d> sub_gt(vio_t_w_i.begin(), vio_t_w_i.end());
    pangolin::glDrawLineStrip(sub_gt);

    if (curr_vis_data) {
      for (const auto &p : curr_vis_data->states)
        for (const auto &t_i_c : calib.T_i_c) render_camera((p * t_i_c).matrix(), 2.0, state_color, 0.1);

      for (const auto &p : curr_vis_data->frames)
        for (const auto &t_i_c : calib.T_i_c) render_camera((p * t_i_c).matrix(), 2.0, pose_color, 0.1);

      for (const auto &t_i_c : calib.T_i_c)
        render_camera((curr_vis_data->states.back() * t_i_c).matrix(), 2.0, cam_color, 0.1);

      glColor3ubv(pose_color);
      pangolin::glDrawPoints(curr_vis_data->points);
    }

    pangolin::glDrawAxis(Sophus::SE3d().matrix(), 1.0);
  }

  OpticalFlowInput::Ptr last_img_data{};
  void update_last_image(OpticalFlowInput::Ptr &data) { last_img_data = data; }
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
      plotter->AddSeries("$0", "$8", pangolin::DrawingModeLine, pangolin::Colour::Green(), "gyro bias y",
                         &vio_data_log);
      plotter->AddSeries("$0", "$9", pangolin::DrawingModeLine, pangolin::Colour::Blue(), "gyro bias z", &vio_data_log);
    }

    if (show_est_ba) {
      plotter->AddSeries("$0", "$10", pangolin::DrawingModeLine, pangolin::Colour::Red(), "accel bias x",
                         &vio_data_log);
      plotter->AddSeries("$0", "$11", pangolin::DrawingModeLine, pangolin::Colour::Green(), "accel bias y",
                         &vio_data_log);
      plotter->AddSeries("$0", "$12", pangolin::DrawingModeLine, pangolin::Colour::Blue(), "accel bias z",
                         &vio_data_log);
    }

    if (last_img_data) {
      double t = last_img_data->t_ns * 1e-9;
      plotter->AddMarker(pangolin::Marker::Vertical, t, pangolin::Marker::Equal, pangolin::Colour::White());
    }
  }
};
}  // namespace xrt::auxiliary::tracking::slam
