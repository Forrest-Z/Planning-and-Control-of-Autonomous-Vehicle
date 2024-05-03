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
#include "FrenetPath.h"
#include "path_data.h"
#include "speed_data.h"
#include "gridded_path_time_graph.h"

/**
 * @class PathTimeHeuristicOptimizer
 * @brief PathTimeHeuristicOptimizer does ST graph speed planning with dynamic
 * programming algorithm.
 */
class PathTimeHeuristicOptimizer
{
public:
  explicit PathTimeHeuristicOptimizer(const StGraphData &st_graph_data, const std::vector<const Obstacle *> &obstacles);
  bool Process(const PathData &path_data, const TrajectoryPoint &init_point, SpeedData *const speed_data);

  bool SearchPathTimeGraph(SpeedData *speed_data) const;

private:
  TrajectoryPoint init_point_;
  SL_Boundary adc_sl_boundary_;

  StGraphData st_graph_data_;
  std::vector<const Obstacle *> obstacles_;
};