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
#pragma once

#include <Eigen/Dense>

#include <pangolin/display/image_view.h>
#include <pangolin/gl/gldraw.h>
#include <pangolin/gl/glfont.h>

#include <basalt/utils/vis_matrices.h>
#include <basalt/vi_estimator/vio_estimator.h>
#include <pangolin/var/var.h>
#include <pangolin/var/varvaluegeneric.h>
#include <basalt/utils/sophus_utils.hpp>
#include <tuple>

const uint8_t cam_color[3]{250, 0, 125};
const uint8_t state_color[3]{250, 0, 26};
const uint8_t pose_color[3]{0, 50, 255};
const uint8_t gt_color[3]{0, 171, 47};
const float MIN_DEPTH_COLOR[3]{0.27, 0.79, 1};      // blue
const float MAX_DEPTH_COLOR[3]{1, 0.1, 0.42};       // pink
const uint8_t MIN_DEPTH_COLOR_UB[3]{69, 201, 255};  // blue
const uint8_t MAX_DEPTH_COLOR_UB[3]{255, 26, 107};  // pink

inline void render_camera(const Eigen::Matrix4d& T_w_c, float lineWidth, const uint8_t* color, float sizeFactor,
                          bool show_ids = false, size_t frame_idx = 0, const uint8_t* idx_color = nullptr,
                          bool show_fwd = false) {
  const float sz = sizeFactor;
  const float width = 640, height = 480, fx = 500, fy = 500, cx = 320, cy = 240;

  Eigen::aligned_vector<Eigen::Vector3f> lines = {{0, 0, 0},
                                                  {sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz},
                                                  {0, 0, 0},
                                                  {sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz},
                                                  {0, 0, 0},
                                                  {sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz},
                                                  {0, 0, 0},
                                                  {sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz},
                                                  {sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz},
                                                  {sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz},
                                                  {sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz},
                                                  {sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz},
                                                  {sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz},
                                                  {sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz},
                                                  {sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz},
                                                  {sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz}};
  if (show_fwd) {
    lines.emplace_back(0, 0, 0);
    lines.emplace_back(0, 0, 1);
  }

  glPushMatrix();
  glMultMatrixd(T_w_c.data());
  glColor3ubv(color);
  glLineWidth(lineWidth);
  pangolin::glDrawLines(lines);
  if (show_ids) {
    glColor3ubv(idx_color);
    pangolin::GlFont::I().Text("%d", frame_idx).Draw(0, 0, -0.01F);
  }
  glPopMatrix();
}

inline void getcolor(float p, float np, float& r, float& g, float& b) {
  float inc = 4.0 / np;
  float x = p * inc;
  r = 0.0f;
  g = 0.0f;
  b = 0.0f;

  if ((0 <= x && x <= 1) || (5 <= x && x <= 6))
    r = 1.0f;
  else if (4 <= x && x <= 5)
    r = x - 4;
  else if (1 <= x && x <= 2)
    r = 1.0f - (x - 1);

  if (1 <= x && x <= 3)
    g = 1.0f;
  else if (0 <= x && x <= 1)
    g = x - 0;
  else if (3 <= x && x <= 4)
    g = 1.0f - (x - 3);

  if (3 <= x && x <= 5)
    b = 1.0f;
  else if (2 <= x && x <= 3)
    b = x - 2;
  else if (5 <= x && x <= 6)
    b = 1.0f - (x - 5);
}

inline std::tuple<float, float, float> color_lerp(float t,                               //
                                                  const float min[3] = MIN_DEPTH_COLOR,  //
                                                  const float max[3] = MAX_DEPTH_COLOR   //
) {
  return {min[0] + t * (max[0] - min[0]),  //
          min[1] + t * (max[1] - min[1]),  //
          min[2] + t * (max[2] - min[2])};
}

inline std::tuple<uint8_t, uint8_t, uint8_t> color_lerp_ub(float t,                                      //
                                                           const uint8_t minub[3] = MIN_DEPTH_COLOR_UB,  //
                                                           const uint8_t maxub[3] = MAX_DEPTH_COLOR_UB   //
) {
  float min[3] = {minub[0] / 255.0F, minub[1] / 255.0F, minub[2] / 255.0F};
  float max[3] = {maxub[0] / 255.0F, maxub[1] / 255.0F, maxub[2] / 255.0F};
  auto [r, g, b] = color_lerp(t, min, max);
  return {uint8_t(r * 255.0F), uint8_t(g * 255.0F), uint8_t(b * 255.0F)};
}

template <typename P, int N, class Allocator>
void glDrawCirclePerimeters(const std::vector<Eigen::Matrix<P, N, 1>, Allocator>& points, float radius = 5.0) {
  for (auto& p : points) {
    pangolin::glDrawCirclePerimeter((GLfloat)p(0), (GLfloat)p(1), (GLfloat)radius);
  }
}

namespace basalt::vis {

using Eigen::MatrixXf;
using Eigen::Vector2d;
using Eigen::Vector2f;
using Eigen::Vector4d;
using pangolin::ImageView;
using pangolin::META_FLAG_READONLY;
using pangolin::Var;
using pangolin::View;
using std::shared_ptr;
using std::string;
using std::vector;
using Button = Var<std::function<void(void)>>;

extern pangolin::GlFont SMALL_FONT;

const uint8_t BLUE[4]{0x21, 0x96, 0xF3, 0xFF};
const uint8_t GREEN[4]{0x4C, 0xAF, 0x50, 0xFF};
const uint8_t RED[4]{0xF4, 0x43, 0x36, 0xFF};

struct SelectionNode {
  bool is_range;
  size_t a;
  size_t b;

  bool contains(size_t n) const { return is_range ? a <= n && n <= b : n == a; }
};
using Selection = std::vector<SelectionNode>;

//! Parse a set of numbers described in @p str. Example inputs: "1,3,5-10", "1000-2000,3,5-7"
Selection parse_selection(const std::string& str);

bool is_selected(const Selection& selection, size_t n);

struct VIOUIBase {
  static constexpr int UI_WIDTH_PIX = 200;
  const pangolin::Attach UI_WIDTH = pangolin::Attach::Pix(UI_WIDTH_PIX);

  View* img_view_display;
  View* plot_display;
  View* blocks_display;
  vector<shared_ptr<ImageView>> img_view;
  bool show_blocks = false;
  Selection highlights{};
  VioConfig config;
  Calibration<double> calib;
  OpticalFlowBase::Ptr opt_flow;
  VioEstimatorBase::Ptr vio;

  Var<int> show_frame{"ui.show_frame", 0, META_FLAG_READONLY};

  Var<bool> show_flow{"ui.show_flow", false, true};
  Var<bool> show_responses{"ui.show_responses", false, true};
  Var<bool> show_tracking_guess{"ui.show_tracking_guess", false, true};
  Var<bool> show_matching_guess{"ui.show_matching_guess", false, true};
  Var<bool> show_recall_guess{"ui.show_recall_guess", false, true};
  Var<bool> show_obs{"ui.show_obs", true, true};
  Var<bool> show_ids{"ui.show_ids", false, true};
  Var<bool> show_depth{"ui.show_depth", false, true};

  Var<std::string> highlight_landmarks{"ui.Highlight", ""};
  Var<bool> filter_highlights{"ui.filter_highlights", false, true};
  Var<bool> show_highlights{"ui.show_highlights", false, true};
  Var<bool> follow_highlight{"ui.follow_highlight", false, true};
  Button highlight_frame_btn{"ui.highlight_frame", [this]() { highligh_frame(); }};

  Button toggle_blocks_btn{"ui.toggle_blocks", [this]() { toggle_blocks(); }};
  Var<std::string> mat_name{"ui.mat_name", "Jr", META_FLAG_READONLY};
  Var<int> mat_to_show{"ui.mat_to_show", (int)UIMAT::JR, (int)UIMAT::JR, (int)UIMAT::COUNT - 1};

  Var<bool> show_block_vals{"ui.show_block_vals", false, true};

  Button take_ltkf_btn{"ui.Take Keyframe", [this]() { take_ltkf(); }};

  Var<bool> show_grid{"ui.show_grid", false, true};
  Var<bool> show_safe_radius{"ui.show_safe_radius", false, true};
  Var<bool> show_cam0_proj{"ui.show_cam0_proj", false, true};
  Var<bool> show_masks{"ui.show_masks", false, true};

  Var<bool> show_guesses{"ui.Show matching guesses", false, true};
  Var<bool> show_same_pixel_guess{"ui.SAME_PIXEL", true, true};
  Var<bool> show_reproj_avg_depth_guess{"ui.REPROJ_AVG_DEPTH", true, true};
  Var<bool> show_reproj_fix_depth_guess{"ui.REPROJ_FIX_DEPTH", true, true};
  Var<double> fixed_depth{"ui.FIX_DEPTH", 2, 0, 3};
  Var<bool> show_active_guess{"ui.Active Guess", true, true};

  Var<double> depth_guess{"ui.depth_guess", 2, META_FLAG_READONLY};

  Var<bool> show_est_pos{"ui.show_est_pos", true, true};
  Var<bool> show_est_vel{"ui.show_est_vel", false, true};
  Var<bool> show_est_bg{"ui.show_est_bg", false, true};
  Var<bool> show_est_ba{"ui.show_est_ba", false, true};

  Var<bool> follow{"ui.follow", true, true};

  virtual VioVisualizationData::Ptr get_curr_vis_data() = 0;

  bool is_highlighted(size_t lmid) const { return vis::is_selected(highlights, lmid); }
  bool highligh_frame();
  bool toggle_blocks();
  bool take_ltkf();
  void do_show_flow(size_t cam_id);
  void do_show_highlights(size_t cam_id);
  void do_show_tracking_guess(size_t cam_id, size_t frame_id, const VioVisualizationData::Ptr& prev_vis_data);
  void do_show_tracking_guess_vio(size_t cam_id, size_t frame_id, const VioDatasetPtr& vio_dataset,
                                  const std::unordered_map<int64_t, VioVisualizationData::Ptr>& vis_map);
  void do_show_recall_guesses(size_t cam_id);
  void do_show_matching_guesses(size_t cam_id);
  void do_show_masks(size_t cam_id);
  void do_show_cam0_proj(size_t cam_id, double depth_guess);
  void do_show_grid();
  void do_show_safe_radius();
  void do_show_guesses(size_t cam_id);
  void do_show_obs(size_t cam_id);
  void draw_blocks_overlay(pangolin::ImageView& blocks_view);
  void draw_jacobian_overlay(pangolin::ImageView& blocks_view, const UIJacobians& uij);
  void draw_hessian_overlay(pangolin::ImageView& blocks_view, const UIHessians& uih);
  bool do_toggle_blocks(pangolin::View* blocks_display, pangolin::View* plot_display, pangolin::View* img_view_display,
                        pangolin::Attach UI_WIDTH);
  void do_show_blocks(const shared_ptr<ImageView>& blocks_view);
  void do_show_hessian(const shared_ptr<ImageView>& blocks_view, UIHessians& uih);
  void do_show_jacobian(const shared_ptr<ImageView>& blocks_view, UIJacobians& uij);
  bool do_follow_highlight(bool smooth_zoom);

  void do_render_camera(const Sophus::SE3d& T_w_c, size_t i, size_t ts, const uint8_t* color);
};

}  // namespace basalt::vis
