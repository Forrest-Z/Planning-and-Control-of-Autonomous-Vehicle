/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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
  Modification: Modify the input of the constructor
**/

#ifndef TRAJECTORY_COMBINER_H
#define TRAJECTORY_COMBINER_H
#include "cartesian_frenet_conversion.h"
#include "lattice_trajectory1d.h"
#include "trajectory_evaluator.h"
#include "path_matcher.h"
#include <vector>
#include <cmath>
#include <chrono>
#include <queue>
#include <string>

class TrajectoryCombiner
{
public:
  TrajectoryCombiner();
  ~TrajectoryCombiner();
  DiscretizedTrajectory Combine(const std::vector<double> &accumulated_s, const Curve1d &lon_trajectory,
                                const Curve1d &lat_trajectory, const std::vector<ReferencePoint> &reference_points,
                                const double &init_relative_time);
};
#endif