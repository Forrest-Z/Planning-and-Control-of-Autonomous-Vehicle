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

#include "dp_road_graph.h"
#include <ros/ros.h>

class DpPolyPathOptimizer
{
public:
  DpPolyPathOptimizer();
  DpPolyPathOptimizer(const std::vector<const Obstacle*>& obstacles, const nav_msgs::Odometry& vehicle_pos);
  ~DpPolyPathOptimizer();

  bool Process(const SpeedData& speed_data, const ReferenceLine& reference_line, const TrajectoryPoint& init_point,
               PathData* const path_data);

private:
  nav_msgs::Odometry vehicle_pos_;
  std::vector<const Obstacle*> obstacles_;
};
