/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file
 * Modified function input and used only some functions
 **/
#pragma once

#include <vector>
#include "boundarys.h"
#include "path_data.h"
#include "speed_data.h"
#include "qp_frenet_frame.h"
#include "qp_solver.h"
#include "spline_1d_solver.h"

class QpSplinePathGenerator
{
public:
  QpSplinePathGenerator(Spline1dSolver *spline_solver, const ReferenceLine &reference_line);

  void SetChangeLane(bool is_change_lane_path);

  bool Generate(const std::vector<const Obstacle *> &obstacles, const SpeedData &speed_data,
                const TrajectoryPoint &init_point, const double boundary_extension, bool is_final_attempt,
                PathData *const path_data);

private:
  bool InitSpline(const double start_s, const double end_s);

  bool AddConstraint(const QpFrenetFrame &qp_frenet_frame, const double boundary_extension);

  void AddKernel();

  void AddHistoryPathKernel();

  bool Solve();

private:
  Spline1dSolver *spline_solver_ = nullptr;
  const ReferenceLine &reference_line_;

  TrajectoryPoint init_trajectory_point_;
  FrenetFramePoint init_frenet_point_;

  std::vector<double> knots_;
  std::vector<double> evaluated_s_;

  const DiscretizedPath *last_discretized_path_ = nullptr;
  bool is_change_lane_path_ = false;
  double ref_l_ = 0.0;

};
