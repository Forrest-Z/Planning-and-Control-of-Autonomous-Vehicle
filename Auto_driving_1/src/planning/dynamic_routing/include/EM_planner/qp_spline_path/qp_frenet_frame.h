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

#include <utility>
#include <vector>
#include <Eigen/Core>
#include "boundarys.h"
#include "speed_data.h"
#include "reference_line.h"
#include "Obstacle.h"

class QpFrenetFrame
{
public:
  QpFrenetFrame(const ReferenceLine &reference_line, const SpeedData &speed_data,
                const FrenetFramePoint &init_frenet_point, const double time_resolution,
                const std::vector<double> &evaluated_s);
  virtual ~QpFrenetFrame() = default;

  bool Init(const std::vector<const Obstacle *> &obstacles);

  const std::vector<std::pair<double, double>> &GetMapBound() const;
  const std::vector<std::pair<double, double>> &GetStaticObstacleBound() const;
  const std::vector<std::pair<double, double>> &GetDynamicObstacleBound() const;

private:
  bool CalculateDiscretizedVehicleLocation();

  bool MapDynamicObstacleWithDecision(const Obstacle &obstacle);

  bool MapStaticObstacleWithDecision(const Obstacle &obstacle);

  bool MapNudgePolygon(const common::math::Polygon2d &polygon, const ObjectNudge &nudge,
                       std::vector<std::pair<double, double>> *const bound_map);

  bool MapNudgeLine(const SLPoint &start, const SLPoint &end, const ObjectNudge::Type type,
                    std::vector<std::pair<double, double>> *const constraint);

  std::pair<double, double> MapLateralConstraint(const SLPoint &start, const SLPoint &end,
                                                 const ObjectNudge::Type nudge_type, const double s_start,
                                                 const double s_end);

  std::pair<uint32_t, uint32_t> FindInterval(const double start, const double end) const;

  bool CalculateHDMapBound();

  bool CalculateObstacleBound(const std::vector<const Obstacle *> &obstacles);

  uint32_t FindIndex(const double s) const;

private:
  const ReferenceLine &reference_line_;
  const SpeedData &speed_data_;

  FrenetFramePoint init_frenet_point_;

  double feasible_longitudinal_upper_bound_ = 0.0;
  double start_s_ = 0.0;
  double end_s_ = 0.0;
  double time_resolution_ = 0.1;

  std::vector<double> evaluated_s_;
  std::vector<SpeedPoint> discretized_vehicle_location_;
  std::vector<std::pair<double, double>> hdmap_bound_;
  std::vector<std::pair<double, double>> static_obstacle_bound_;
  std::vector<std::pair<double, double>> dynamic_obstacle_bound_;
};