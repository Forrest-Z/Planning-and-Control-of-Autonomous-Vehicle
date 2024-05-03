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
#include "Obstacle.h"
#include "path_struct.h"
#include "comparable_cost.h"
#include "speed_data.h"
#include "box2d.h"
#include "boundarys.h"
#include "reference_line.h"
#include "path_struct.h"

class TrajectoryCost
{
public:
  TrajectoryCost() = default;
  explicit TrajectoryCost(const VehicleConfig vehicle_config, const ReferenceLine &reference_line,
                          const bool is_change_lane_path, const std::vector<const Obstacle *> &obstacles,
                          const SpeedData &heuristic_speed_data, const SLPoint &init_sl_point);

  ComparableCost Calculate(const QuinticPolynomialCurve1d &curve, const double start_s, const double end_s,
                           const uint32_t curr_level, const uint32_t total_level);

private:
  ComparableCost CalculatePathCost(const QuinticPolynomialCurve1d &curve, const double start_s, const double end_s,
                                   const uint32_t curr_level, const uint32_t total_level);
  ComparableCost CalculateStaticObstacleCost(const QuinticPolynomialCurve1d &curve, const double start_s,
                                             const double end_s);
  ComparableCost CalculateDynamicObstacleCost(const QuinticPolynomialCurve1d &curve, const double start_s,
                                              const double end_s) const;
  ComparableCost GetCostBetweenObsBoxes(const Box2d &ego_box, const Box2d &obstacle_box) const;

  ComparableCost GetCostFromObsSL(const double adc_s, const double adc_l, const SL_Boundary &obs_sl_boundary);

  Box2d GetBoxFromSLPoint(const SLPoint &sl, const double dl) const;

  bool IsOffRoad(const double ref_s, const double l, const double dl, const bool is_change_lane_path);

  bool is_change_lane_path_ = false; //是变道

  const VehicleConfig vehicle_config_;
  const ReferenceLine *reference_line_ = nullptr;
  SpeedData heuristic_speed_data_;
  const SLPoint init_sl_point_;

  uint32_t num_of_time_stamps_ = 0;
  std::shared_ptr<SL_Boundary> ptr_obstacle;
  // 障碍物
  std::vector<std::vector<Box2d>> dynamic_obstacle_boxes_;
  std::vector<double> obstacle_probabilities_;
  std::vector<SL_Boundary> static_obstacle_sl_boundaries_;
};