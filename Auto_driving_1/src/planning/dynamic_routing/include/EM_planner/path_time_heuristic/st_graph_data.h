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
#include <tuple>
#include <vector>
#include "boundarys.h"
#include "speed_limit.h"
#include "FrenetPath.h"
#include "path_struct.h"

constexpr double kObsSpeedIgnoreThreshold = 100.0;

class StGraphData
{
public:
  StGraphData() = default;

  void LoadData(const std::vector<ST_Boundary> &st_boundaries, const double min_s_on_st_boundaries,
                const TrajectoryPoint &init_point, const planning::SpeedLimit &speed_limit, const double cruise_speed,
                const double path_data_length, const double total_time_by_conf);

  bool is_initialized() const
  {
    return init_;
  }

  const std::vector<ST_Boundary> &st_boundaries() const;

  double min_s_on_st_boundaries() const;

  const TrajectoryPoint &init_point() const;

  const planning::SpeedLimit &speed_limit() const;

  double cruise_speed() const;

  double path_length() const;

  double total_time_by_conf() const;

  bool SetSTDrivableBoundary(const std::vector<std::tuple<double, double, double>> &s_boundary,
                             const std::vector<std::tuple<double, double, double>> &v_obs_info);

private:
  bool init_ = false;
  std::vector<ST_Boundary> st_boundaries_;
  double min_s_on_st_boundaries_ = 0.0;
  TrajectoryPoint init_point_;
  planning::SpeedLimit speed_limit_;
  double cruise_speed_ = 0.0;
  double path_data_length_ = 0.0;
  double path_length_by_conf_ = 0.0;
  double total_time_by_conf_ = 0.0;
};