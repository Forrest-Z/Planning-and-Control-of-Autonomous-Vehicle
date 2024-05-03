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
#include "dp_st_cost.h"
#include <algorithm>
#include <limits>

namespace
{
  constexpr double kInf = std::numeric_limits<double>::infinity();
}

DpStCost::DpStCost(const double total_t, const double total_s, const std::vector<const Obstacle *> &obstacles,
                   const TrajectoryPoint &init_point)
    : obstacles_(obstacles), init_point_(init_point), unit_t_(Config_.unit_t), total_s_(total_s)
{
  int index = 0;
  for (const auto &obstacle : obstacles)
  {
    boundary_map_[obstacle->path_st_boundary().id()] = index++;
  }

  AddToKeepClearRange(obstacles);

  const auto dimension_t = static_cast<uint32_t>(std::ceil(total_t / static_cast<double>(unit_t_))) + 1;
  boundary_cost_.resize(obstacles_.size());
  for (auto &vec : boundary_cost_)
  {
    vec.resize(dimension_t, std::make_pair(-1.0, -1.0));
  }
  accel_cost_.fill(-1.0);
  jerk_cost_.fill(-1.0);
}

void DpStCost::AddToKeepClearRange(const std::vector<const Obstacle *> &obstacles)
{
  for (const auto &obstacle : obstacles)
  {
    if (obstacle->path_st_boundary().boundary_type() != ST_Boundary::BoundaryType::KEEP_CLEAR)
    {
      continue;
    }

    double start_s = obstacle->path_st_boundary().min_s_;
    double end_s = obstacle->path_st_boundary().max_s_;
    keep_clear_range_.emplace_back(start_s, end_s);
  }
  SortAndMergeRange(&keep_clear_range_);
}

void DpStCost::SortAndMergeRange(std::vector<std::pair<double, double>> *keep_clear_range)
{
  if (!keep_clear_range || keep_clear_range->empty())
  {
    return;
  }
  std::sort(keep_clear_range->begin(), keep_clear_range->end());
  size_t i = 0;
  size_t j = i + 1;
  while (j < keep_clear_range->size())
  {
    if (keep_clear_range->at(i).second < keep_clear_range->at(j).first)
    {
      ++i;
      ++j;
    }
    else
    {
      keep_clear_range->at(i).second = std::max(keep_clear_range->at(i).second, keep_clear_range->at(j).second);
      ++j;
    }
  }
  keep_clear_range->resize(i + 1);
}

bool DpStCost::InKeepClearRange(double s) const
{
  for (const auto &p : keep_clear_range_)
  {
    if (p.first <= s && p.second >= s)
    {
      return true;
    }
  }
  return false;
}

//当前节点(相对时间t，累积距离s)和障碍物运动轨迹的开销
//每个障碍物在未来的时间间隔内(例如5s，每0.1s就有一个采样的位置s)都有它的运动轨迹，也就是运动位置s_x。
//那么在相对时间t和累积距离s时刻，无人车会不会和障碍物该时刻相撞呢？这部分就需要计算无人车和障碍物在t时刻的位置信息，也就是位置cost。
//障碍物位置开销思路比较简单，循环每个障碍物，计算t时刻障碍物st边界框的上界和下届，只需要无人车的位置(t,s)与边界框不重合即可。
double DpStCost::GetObstacleCost(const StGraphPoint &st_graph_point)
{
  const double s = st_graph_point.point().s();
  const double t = st_graph_point.point().t();

  double cost = 0.0;

  for (const auto *obstacle : obstacles_)
  {

    // Not applying obstacle approaching cost to virtual obstacle like created
    // stop fences
    if (obstacle->IsVirtual())
    {
      continue;
    }

    auto boundary = obstacle->path_st_boundary();
    if (boundary.min_s_ > Config_.FLAGS_speed_lon_decision_horizon)
    {
      continue;
    }

    if (t < boundary.min_t_ || t > boundary.max_t_)
    {
      continue;
    }
    // std::cout << "upper_points_:"
    //           << boundary.getUpper_points().size() << "\n";
    if (boundary.IsPointInBoundary(st_graph_point.point()))
    {
      return kInf;
    }
    double s_upper = 0.0;
    double s_lower = 0.0;

    int boundary_index = boundary_map_[boundary.id()];
    if (boundary_cost_[boundary_index][st_graph_point.index_t()].first < 0.0)
    {
      boundary.GetBoundarySRange(t, &s_upper, &s_lower);
      boundary_cost_[boundary_index][st_graph_point.index_t()] = std::make_pair(s_upper, s_lower);
    }
    else
    {
      s_upper = boundary_cost_[boundary_index][st_graph_point.index_t()].first;
      s_lower = boundary_cost_[boundary_index][st_graph_point.index_t()].second;
    }
    if (s < s_lower)
    {
      const double follow_distance_s = Config_.safe_distance;
      if (s + follow_distance_s < s_lower)
      {
        continue;
      }
      else
      {
        auto s_diff = follow_distance_s - s_lower + s;
        cost += Config_.obstacle_weight * Config_.default_obstacle_cost * s_diff * s_diff;
      }
    }
    else if (s > s_upper)
    {
      const double overtake_distance_s = 20.0; // kDpSafetyDistance
      if (s > s_upper + overtake_distance_s)
      { // or calculated from velocity
        continue;
      }
      else
      {
        auto s_diff = overtake_distance_s + s_upper - s;
        cost += Config_.obstacle_weight * Config_.default_obstacle_cost * s_diff * s_diff;
      }
    }
  }

  return cost * unit_t_;
}

double DpStCost::GetSpatialPotentialCost(const StGraphPoint &point)
{
  return (total_s_ - point.point().s()) * Config_.spatial_potential_penalty;
}

double DpStCost::GetReferenceCost(const STPoint &point, const STPoint &reference_point) const
{
  return Config_.reference_weight * (point.s() - reference_point.s()) * (point.s() - reference_point.s()) * unit_t_;
}

double DpStCost::GetSpeedCost(const STPoint &first, const STPoint &second, const double speed_limit,
                              const double cruise_speed) const
{
  double cost = 0.0;
  const double speed = (second.s() - first.s()) / unit_t_;
  if (speed < 0)
  {
    return kInf;
  }

  const double max_adc_stop_speed = Config_.max_abs_speed_when_stopped;
  if (speed < max_adc_stop_speed && InKeepClearRange(second.s()))
  {
    // first.s in range
    cost += Config_.keep_clear_low_speed_penalty * unit_t_ * Config_.default_speed_cost;
  }

  double det_speed = (speed - speed_limit) / speed_limit;
  if (det_speed > 0)
  {
    cost += Config_.exceed_speed_penalty * Config_.default_speed_cost * (det_speed * det_speed) * unit_t_;
  }
  else if (det_speed < 0)
  {
    cost += Config_.low_speed_penalty * Config_.default_speed_cost * -det_speed * unit_t_;
  }

  if (Config_.FLAGS_enable_dp_reference_speed)
  {
    double diff_speed = speed - cruise_speed;
    cost += Config_.reference_speed_penalty * Config_.default_speed_cost * fabs(diff_speed) * unit_t_;
  }

  return cost;
}

double DpStCost::GetAccelCost(const double accel)
{
  double cost = 0.0;
  static constexpr double kEpsilon = 0.1;
  static constexpr size_t kShift = 100;
  const size_t accel_key = static_cast<size_t>(accel / kEpsilon + 0.5 + kShift);
  //   DCHECK_LT(accel_key, accel_cost_.size());
  if (accel_key >= accel_cost_.size())
  {
    return kInf;
  }

  if (accel_cost_.at(accel_key) < 0.0)
  {
    const double accel_sq = accel * accel;
    double max_acc = Config_.max_acceleration;
    double max_dec = Config_.max_deceleration;
    double accel_penalty = accel_penalty;
    double decel_penalty = decel_penalty;

    if (accel > 0.0)
    {
      cost = accel_penalty * accel_sq;
    }
    else
    {
      cost = decel_penalty * accel_sq;
    }
    cost += accel_sq * decel_penalty * decel_penalty / (1 + std::exp(1.0 * (accel - max_dec))) +
            accel_sq * accel_penalty * accel_penalty / (1 + std::exp(-1.0 * (accel - max_acc)));
    accel_cost_.at(accel_key) = cost;
  }
  else
  {
    cost = accel_cost_.at(accel_key);
  }
  return cost * unit_t_;
}

double DpStCost::GetAccelCostByThreePoints(const STPoint &first, const STPoint &second, const STPoint &third)
{
  double accel = (first.s() + third.s() - 2 * second.s()) / (unit_t_ * unit_t_);
  return GetAccelCost(accel);
}

double DpStCost::GetAccelCostByTwoPoints(const double pre_speed, const STPoint &pre_point, const STPoint &curr_point)
{
  double current_speed = (curr_point.s() - pre_point.s()) / unit_t_;
  double accel = (current_speed - pre_speed) / unit_t_;
  return GetAccelCost(accel);
}

double DpStCost::JerkCost(const double jerk)
{
  double cost = 0.0;
  static constexpr double kEpsilon = 0.1;
  static constexpr size_t kShift = 200;
  const size_t jerk_key = static_cast<size_t>(jerk / kEpsilon + 0.5 + kShift);
  if (jerk_key >= jerk_cost_.size())
  {
    return kInf;
  }

  if (jerk_cost_.at(jerk_key) < 0.0)
  {
    double jerk_sq = jerk * jerk;
    if (jerk > 0)
    {
      cost = Config_.positive_jerk_coeff * jerk_sq * unit_t_;
    }
    else
    {
      cost = Config_.negative_jerk_coeff * jerk_sq * unit_t_;
    }
    jerk_cost_.at(jerk_key) = cost;
  }
  else
  {
    cost = jerk_cost_.at(jerk_key);
  }

  // TODO(All): normalize to unit_t_
  return cost;
}

double DpStCost::GetJerkCostByFourPoints(const STPoint &first, const STPoint &second, const STPoint &third,
                                         const STPoint &fourth)
{
  double jerk = (fourth.s() - 3 * third.s() + 3 * second.s() - first.s()) / (unit_t_ * unit_t_ * unit_t_);
  return JerkCost(jerk);
}

double DpStCost::GetJerkCostByTwoPoints(const double pre_speed, const double pre_acc, const STPoint &pre_point,
                                        const STPoint &curr_point)
{
  const double curr_speed = (curr_point.s() - pre_point.s()) / unit_t_;
  const double curr_accel = (curr_speed - pre_speed) / unit_t_;
  const double jerk = (curr_accel - pre_acc) / unit_t_;
  return JerkCost(jerk);
}

double DpStCost::GetJerkCostByThreePoints(const double first_speed, const STPoint &first, const STPoint &second,
                                          const STPoint &third)
{
  const double pre_speed = (second.s() - first.s()) / unit_t_;
  const double pre_acc = (pre_speed - first_speed) / unit_t_;
  const double curr_speed = (third.s() - second.s()) / unit_t_;
  const double curr_acc = (curr_speed - pre_speed) / unit_t_;
  const double jerk = (curr_acc - pre_acc) / unit_t_;
  return JerkCost(jerk);
}