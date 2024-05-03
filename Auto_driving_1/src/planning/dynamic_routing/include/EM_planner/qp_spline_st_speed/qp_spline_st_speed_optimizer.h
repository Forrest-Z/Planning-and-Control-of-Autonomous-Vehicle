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
  Modification: Only some functions are referenced
**/
#pragma once
#include "active_set_spline_1d_solver.h"
#include "trajectoryPoint.h"
#include "reference_line_info.h"
#include "speed_data.h"
#include "path_data.h"
#include "osqp_spline_1d_solver.h"
#include "st_graph_data.h"

class QpSplineStSpeedOptimizer
{
public:
  explicit QpSplineStSpeedOptimizer();

  bool Process(const SL_Boundary &adc_sl_boundary,
               const StGraphData &st_graph_data,
               const PathData &path_data,
               const TrajectoryPoint &init_point,
               ReferenceLineInfo &reference_line_info,
               SpeedData *const speed_data);

private:
  std::unique_ptr<Spline1dSolver> spline_solver_;
  ReferenceLineInfo reference_line_info_;
};
