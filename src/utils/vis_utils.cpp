/**
BSD 3-Clause License

This file is part of the Basalt project.
https://gitlab.com/VladyslavUsenko/basalt.git

Copyright (c) 2023, Collabora Ltd.
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

#include <basalt/optical_flow/optical_flow.h>
#include <basalt/utils/vis_utils.h>
#include <pangolin/gl/glfont.h>

namespace basalt::vis {

using Eigen::MatrixXf;
using Eigen::Vector2d;
using Eigen::Vector2f;
using Eigen::Vector4d;

void show_flow(size_t cam_id, const VioVisualizationData::Ptr& curr_vis_data, const OpticalFlowBase::Ptr& opt_flow,
               bool show_ids) {
  glLineWidth(1.0);
  glColor3f(1.0, 0.0, 0.0);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  const std::vector<Keypoints>& keypoints = curr_vis_data->opt_flow_res->keypoints;
  if (cam_id >= keypoints.size()) return;

  const Keypoints& kp_map = keypoints[cam_id];

  for (const auto& kv : kp_map) {
    MatrixXf transformed_patch = kv.second.linear() * opt_flow->patch_coord;
    transformed_patch.colwise() += kv.second.translation();

    for (int i = 0; i < transformed_patch.cols(); i++) {
      const Vector2f c = transformed_patch.col(i);
      pangolin::glDrawCirclePerimeter(c[0], c[1], 0.5F);
    }

    const Vector2f c = kv.second.translation();

    if (show_ids) pangolin::GlFont::I().Text("%d", kv.first).Draw(5 + c[0], 5 + c[1]);
  }

  pangolin::GlFont::I().Text("Detected %d keypoints", kp_map.size()).Draw(5, 40);
}

void show_tracking_guess(size_t cam_id, size_t frame_id, const VioVisualizationData::Ptr& curr_vis_data,
                         const VioVisualizationData::Ptr& prev_vis_data) {
  if (frame_id < 1) return;

  auto new_kpts = curr_vis_data->opt_flow_res->keypoints[cam_id];
  auto prev_kpts = prev_vis_data->opt_flow_res->keypoints[cam_id];
  auto guess_obs = curr_vis_data->opt_flow_res->tracking_guesses[cam_id];

  std::vector<Vector2f> prev_lines;
  std::vector<Vector2f> prev_points;
  std::vector<Vector2f> guess_lines;
  std::vector<Vector2f> guess_points;
  std::vector<Vector2f> now_points;

  prev_lines.reserve(new_kpts.size());
  prev_points.reserve(new_kpts.size());
  guess_lines.reserve(new_kpts.size());
  guess_points.reserve(new_kpts.size());
  now_points.reserve(new_kpts.size());

  float radius = 3.0F;

  // Draw tracked features in previous frame
  for (auto& [kpid, kpt] : new_kpts) {
    if (prev_kpts.count(kpid) == 0) continue;
    if (guess_obs.count(kpid) == 0) continue;

    auto n = kpt.translation();
    auto p = prev_kpts.at(kpid).translation();
    auto g = guess_obs.at(kpid).translation();

    now_points.emplace_back(n);

    prev_lines.emplace_back(p);
    prev_lines.emplace_back(n);
    prev_points.emplace_back(p);

    guess_lines.emplace_back(g);
    guess_lines.emplace_back(n);
    guess_points.emplace_back(g);
  }

  glColor4f(1, 0.59, 0, 0.9);
  glDrawCirclePerimeters(now_points, radius);

  glColor4f(0.93, 0.42, 0, 0.3);
  pangolin::glDrawLines(prev_lines);
  glDrawCirclePerimeters(prev_points, radius);

  glColor4f(1, 0.59, 0, 0.5);
  pangolin::glDrawLines(guess_lines);
  glDrawCirclePerimeters(guess_points, radius);
}

void show_tracking_guess_vio(size_t cam_id, size_t frame_id, const VioDatasetPtr& vio_dataset,
                             const std::unordered_map<int64_t, VioVisualizationData::Ptr>& vis_map) {
  if (frame_id < 1) return;

  int64_t now_ts = vio_dataset->get_image_timestamps().at(frame_id);
  int64_t prev_ts = vio_dataset->get_image_timestamps().at(frame_id - 1);

  auto now_it = vis_map.find(now_ts);
  auto prev_it = vis_map.find(prev_ts);

  auto end_it = vis_map.end();
  if (now_it == end_it || prev_it == end_it) return;

  const VioVisualizationData::Ptr& curr_vis_data = now_it->second;
  const VioVisualizationData::Ptr& prev_vis_data = prev_it->second;

  show_tracking_guess(cam_id, frame_id, curr_vis_data, prev_vis_data);
}

void show_matching_guesses(size_t cam_id, const VioVisualizationData::Ptr& curr_vis_data) {
  auto new_kpts = curr_vis_data->opt_flow_res->keypoints.at(cam_id);
  auto cam0_kpts = curr_vis_data->opt_flow_res->keypoints.at(0);
  auto guess_obs = curr_vis_data->opt_flow_res->matching_guesses.at(cam_id);

  std::vector<Vector2f> cam0_lines;
  std::vector<Vector2f> cam0_points;
  std::vector<Vector2f> guess_lines;
  std::vector<Vector2f> guess_points;
  std::vector<Vector2f> now_points;

  cam0_lines.reserve(new_kpts.size());
  cam0_points.reserve(new_kpts.size());
  guess_lines.reserve(new_kpts.size());
  guess_points.reserve(new_kpts.size());
  now_points.reserve(new_kpts.size());

  float radius = 3.0F;

  // Draw tracked features in previous frame
  for (auto& [kpid, kpt] : new_kpts) {
    if (cam0_kpts.count(kpid) == 0) continue;
    if (guess_obs.count(kpid) == 0) continue;

    auto n = kpt.translation();
    auto c = cam0_kpts.at(kpid).translation();
    auto g = guess_obs.at(kpid).translation();

    now_points.emplace_back(n);

    cam0_lines.emplace_back(c);
    cam0_lines.emplace_back(n);
    cam0_points.emplace_back(c);

    guess_lines.emplace_back(g);
    guess_lines.emplace_back(n);
    guess_points.emplace_back(g);
  }

  glColor4f(0.12, 0.58, 0.95, 0.9);
  glDrawCirclePerimeters(now_points, radius);

  glColor4f(0, 0.73, 0.83, 0.5);
  pangolin::glDrawLines(cam0_lines);
  glDrawCirclePerimeters(cam0_points, radius);

  glColor4f(0.12, 0.58, 0.95, 0.5);
  pangolin::glDrawLines(guess_lines);
  glDrawCirclePerimeters(guess_points, radius);
}

void show_masks(size_t cam_id, const VioVisualizationData::Ptr& curr_vis_data) {
  glColor4f(0.0, 1.0, 1.0, 0.1);
  for (const Rect& m : curr_vis_data->opt_flow_res->input_images->masks[cam_id].masks) {
    pangolin::glDrawRect(m.x, m.y, m.x + m.w, m.y + m.h);
  }
}

void show_cam0_proj(size_t cam_id, double depth_guess, const VioConfig& config, const Calibration<double>& calib) {
  int C = config.optical_flow_detection_grid_size;

  int w = calib.resolution.at(0).x();
  int x_start = (w % C) / 2;
  int x_stop = x_start + C * (w / C - 1);
  int x_first = x_start + C / 2;
  int x_last = x_stop + C / 2;

  int h = calib.resolution.at(0).y();
  int y_start = (h % C) / 2;
  int y_stop = y_start + C * (h / C - 1);
  int y_first = y_start + C / 2;
  int y_last = y_stop + C / 2;

  std::vector<Vector2d> points;
  auto drawPoint = [&points, &calib, w, h, depth_guess](float u, float v, int j, bool draw_c0_uv) {
    Vector2d ci_uv{u, v};
    Vector2d c0_uv;
    double _;
    bool projected = calib.projectBetweenCams(ci_uv, depth_guess, c0_uv, _, j, 0);
    bool in_bounds = c0_uv.x() >= 0 && c0_uv.x() < w && c0_uv.y() >= 0 && c0_uv.y() < h;
    bool valid = projected && in_bounds;

    // Define color
    GLfloat invalid_color[4] = {1, 0, 0, 0.5};      // red
    GLfloat in_bounds_color[4] = {1, 0.5, 0, 0.5};  // orange
    GLfloat projected_color[4] = {1, 0.9, 0, 0.5};  // yellow
    GLfloat valid_color[4] = {0, 1, 0, 0.5};        // green
    GLfloat* color = invalid_color;
    if (valid) {
      color = valid_color;
    } else if (projected) {
      color = projected_color;
    } else if (in_bounds) {
      color = in_bounds_color;
    }
    glColor4fv(color);

    // Press L key twice in viewer to be able to see out-of-bounds points
    if (projected) {
      points.push_back(c0_uv);
    }

    if (draw_c0_uv) {
      pangolin::glDrawCircle(c0_uv.x(), c0_uv.y(), 2);
    } else {
      pangolin::glDrawCircle(ci_uv.x(), ci_uv.y(), 2);
    }
  };

  if (cam_id == 0) {
    size_t num_cams = calib.resolution.size();
    for (size_t target_cam = 1; target_cam < num_cams; target_cam++) {
#if 1  // Draw perimeter of projected-to-cam0 grid
      int x = x_first;
      int y = y_first;
      for (; x <= x_last; x += C) drawPoint(x, y, target_cam, true);
      for (x = x_last; y <= y_last; y += C) drawPoint(x, y, target_cam, true);
      for (y = y_last; x >= x_first; x -= C) drawPoint(x, y, target_cam, true);
      for (x = x_first; y >= y_first; y -= C) drawPoint(x, y, target_cam, true);

#else  // Draw full projected-to-cam0 grid
      for (int y = x_first; y <= y_last; y += C) {
        for (int x = y_first; x <= x_last; x += C) {
          drawPoint(x, y, target_cam, true);
        }
      }
#endif

      glColor4f(0.0, 1.0, 0.0, 0.5);
      pangolin::glDrawLineLoop(points);
    }
  } else {
    for (int y = y_first; y < h; y += C) {
      for (int x = x_first; x < w; x += C) {
        drawPoint(x, y, cam_id, false);
      }
    }
  }
}

void show_grid(const VioConfig& config, const Calibration<double>& calib) {
  glColor4f(1.0, 0.0, 1.0, 0.25);

  int C = config.optical_flow_detection_grid_size;

  int w = calib.resolution.at(0).x();
  int x_start = (w % C) / 2;
  int x_stop = x_start + C * (w / C - 1);
  int x_end = x_stop + C;

  int h = calib.resolution.at(0).y();
  int y_start = (h % C) / 2;
  int y_stop = y_start + C * (h / C - 1);
  int y_end = y_stop + C;

  std::vector<Vector2f> grid_lines;
  for (int x = x_start; x <= x_end; x += C) {
    grid_lines.emplace_back(x, y_start);
    grid_lines.emplace_back(x, y_end);
  }
  for (int y = y_start; y <= y_end; y += C) {
    grid_lines.emplace_back(x_start, y);
    grid_lines.emplace_back(x_end, y);
  }
  pangolin::glDrawLines(grid_lines);
}

void show_guesses(size_t cam_id, const VioVisualizationData::Ptr& curr_vis_data, const VioConfig& config,
                  const Calibration<double>& calib, bool show_same_pixel_guess, bool show_reproj_fix_depth_guess,
                  bool show_reproj_avg_depth_guess, bool show_active_guess, double fixed_depth) {
  if (cam_id == 0) return;

  const auto keypoints0 = curr_vis_data->projections->at(0);
  const auto keypoints1 = curr_vis_data->projections->at(cam_id);

  double avg_invdepth = 0;
  double num_features = 0;
  for (const auto& cam_projs : *curr_vis_data->projections) {
    for (const Vector4d& v : cam_projs) avg_invdepth += v.z();
    num_features += cam_projs.size();
  }
  bool valid = avg_invdepth > 0 && num_features > 0;
  float default_depth = config.optical_flow_matching_default_depth;
  double avg_depth = valid ? num_features / avg_invdepth : default_depth;

  for (const Vector4d& kp1 : keypoints1) {
    double u1 = kp1.x();
    double v1 = kp1.y();
    // double invdist1 = kp1.z();
    double id1 = kp1.w();

    double u0 = 0;
    double v0 = 0;
    bool found = false;
    for (const Vector4d& kp0 : keypoints0) {  // Find match in keypoints0
      double id0 = kp0.w();
      if (id1 != id0) continue;
      u0 = kp0.x();
      v0 = kp0.y();
      found = true;
      break;
    }

    // Display guess error if this is a stereo feature
    // NOTE: keep in mind that these guesses are not really the guesses
    // used to detect the feature, but the guess we would use if we were
    // to detect the feature right now.
    if (found) {
      // Guess if we were using SAME_PIXEL
      if (show_same_pixel_guess) {
        glColor3f(0, 1, 1);  // Cyan
        pangolin::glDrawLine(u1, v1, u0, v0);
      }

      // Guess if we were using REPROJ_FIX_DEPTH
      if (show_reproj_fix_depth_guess) {
        glColor3f(1, 1, 0);  // Yellow
        auto off = calib.viewOffset({u0, v0}, fixed_depth, 0, cam_id);
        pangolin::glDrawLine(u1, v1, u0 - off.x(), v0 - off.y());
      }

      // Guess if we were using REPROJ_AVG_DEPTH
      if (show_reproj_avg_depth_guess) {
        glColor3f(1, 0, 1);  // Magenta
        auto off = calib.viewOffset({u0, v0}, avg_depth, 0, cam_id);
        pangolin::glDrawLine(u1, v1, u0 - off.x(), v0 - off.y());
      }

      // Guess with the current guess type
      if (show_active_guess) {
        glColor3f(1, 0, 0);  // Red
        Vector2d off{0, 0};
        if (config.optical_flow_matching_guess_type != MatchingGuessType::SAME_PIXEL) {
          off = calib.viewOffset({u0, v0}, curr_vis_data->opt_flow_res->input_images->depth_guess, 0, cam_id);
        }
        pangolin::glDrawLine(u1, v1, u0 - off.x(), v0 - off.y());
      }
    }
  }
}

void show_obs(size_t cam_id, const VioVisualizationData::Ptr& curr_vis_data, const VioConfig& config,
              const Calibration<double>& calib, bool show_same_pixel_guess, bool show_reproj_fix_depth_guess,
              bool show_reproj_avg_depth_guess, bool show_active_guess, double fixed_depth, bool show_ids,
              bool show_depth, bool show_guesses) {
  glLineWidth(1.0);
  glColor3f(1.0, 0.0, 0.0);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  size_t num_cams = calib.resolution.size();
  if (cam_id < num_cams) {
    const auto& points = curr_vis_data->projections->at(cam_id);

    if (!points.empty()) {
      double min_id = points[0][2];
      double max_id = points[0][2];

      for (const auto& points2 : *curr_vis_data->projections) {
        for (const auto& p : points2) {
          min_id = std::min(min_id, p[2]);
          max_id = std::max(max_id, p[2]);
        }
      }

      for (const auto& c : points) {
        double u = c[0];
        double v = c[1];
        double depth = c[2] != 0.0 ? 1.0 / c[2] : std::numeric_limits<double>::infinity();
        int id = c[3];

        double width = calib.resolution.at(0).x();
        double unit_radius = width / 96;
        double radius = unit_radius / depth;

        double min_depth = 1.0 / 3;  // 1/3 comes from how valid_kp is computed in sqrt_keypoint_vio.cpp
        double max_depth = 20;       // And this is arbitrary
        double max_radius = unit_radius / min_depth;
        double min_radius = unit_radius * min_depth;

        bool clamped = depth < min_depth || depth > max_depth;
        double cradius = std::clamp(radius, min_radius, max_radius);

        float t = (cradius - min_radius) / (max_radius - min_radius);
        auto [r, g, b] = color_lerp(t);

        if (clamped) {  // Mark clamped points in UI
          glColor4f(r, g, b, 0.15);
          pangolin::glDrawCircle(u, v, cradius);
          glColor4f(r, g, b, 1);
        } else {
          glColor4f(r, g, b, 1);
          pangolin::glDrawCirclePerimeter(u, v, cradius);
        }

        if (show_ids) pangolin::GlFont::I().Text("%d", id).Draw(u, v);
        if (show_depth) pangolin::GlFont::I().Text("%.3lf m", depth).Draw(u, v + 5);
      }
    }

    if (show_guesses) {
      vis::show_guesses(cam_id, curr_vis_data, config, calib, show_same_pixel_guess, show_reproj_fix_depth_guess,
                        show_reproj_avg_depth_guess, show_active_guess, fixed_depth);
    }

    glColor3f(0.0, 1.0, 0.0);
    pangolin::GlFont::I().Text("Tracked %d points", points.size()).Draw(5, 20);
  }
}

}  // namespace basalt::vis
