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
#include "qp_frenet_frame.h"
#include <iostream>
#include <algorithm>
#include <limits>
#include "math_utils.h"

namespace
{
  constexpr double kEpsilonTol = 1e-6;
  const auto inf = std::numeric_limits<double>::infinity();
} // namespace

QpFrenetFrame::QpFrenetFrame(const ReferenceLine &reference_line, const SpeedData &speed_data,
                             const FrenetFramePoint &init_frenet_point, const double time_resolution,
                             const std::vector<double> &evaluated_s)
    : reference_line_(reference_line), speed_data_(speed_data), init_frenet_point_(init_frenet_point),
      feasible_longitudinal_upper_bound_(reference_line_.Length()),
      start_s_(init_frenet_point.s), end_s_(reference_line.Length()),
      time_resolution_(time_resolution), evaluated_s_(evaluated_s),
      hdmap_bound_{evaluated_s_.size(), std::make_pair(-inf, inf)},
      static_obstacle_bound_{evaluated_s_.size(), std::make_pair(-inf, inf)},
      dynamic_obstacle_bound_{evaluated_s_.size(), std::make_pair(-inf, inf)}
{
  //   DCHECK_LT(start_s_ + kEpsilonTol, end_s_);
  //   DCHECK_GE(evaluated_s_.size(), 2);
}

/* QpFrenetFrame::Init() 根据已知条件，计算对自车横向轨迹的约束：
  1.HDMap和道路参考线对自车横向轨迹的约束
  2.静态动态障碍物及相应的decision对自车横向轨迹的约束
*/
bool QpFrenetFrame::Init(const std::vector<const Obstacle *> &obstacles)
{
  //根据SpeedData，计算自车纵向行经轨迹，存入vector<SpeedPoint>
  //即discretized_vehicle_location
  if (!CalculateDiscretizedVehicleLocation())
  {
    // std::cout << "Fail to calculate discretized vehicle location!";
    return false;
  }

  //根据HDMap和道路参考线，计算自车轨迹的横向约束
  if (!CalculateHDMapBound())
  {
    // std::cout << "Calculate HDMap bound failed.";
    return false;
  }
  //计算静态、动态障碍物及相应的decision对自车轨迹的横向约束
  if (!CalculateObstacleBound(obstacles))
  {
    // std::cout << "Calculate obstacle bound failed!";
    return false;
  }
  return true;
}

const std::vector<std::pair<double, double>> &QpFrenetFrame::GetMapBound() const
{
  return hdmap_bound_;
}

const std::vector<std::pair<double, double>> &QpFrenetFrame::GetStaticObstacleBound() const
{
  return static_obstacle_bound_;
}

const std::vector<std::pair<double, double>> &QpFrenetFrame::GetDynamicObstacleBound() const
{
  return dynamic_obstacle_bound_;
}

// speed_data从何而来兄弟们
bool QpFrenetFrame::CalculateDiscretizedVehicleLocation()
{
  for (double relative_time = 0.0; relative_time < speed_data_.TotalTime(); relative_time += time_resolution_)
  {
    SpeedPoint veh_point;
    if (!speed_data_.EvaluateByTime(relative_time, &veh_point))
    {
      // std::cout << "Fail to get speed point at relative time " << relative_time;
      return false;
    }
    veh_point.set_t(relative_time);
    discretized_vehicle_location_.push_back(std::move(veh_point));
  }
  return true;
}

/*
MapDynamicObstacleWithDecision()也是和静态障碍物大体相似的思路。根据由SpeedData求得的自车位置信息discretized_vehicle_location_，
按照时间戳一一匹配动态障碍物的轨迹点（应该是预测轨迹），计算该点处的障碍物bounding box对自车横向轨迹的影响。
*/
bool QpFrenetFrame::MapDynamicObstacleWithDecision(const Obstacle &obstacle)
{
  if (!obstacle.HasLateralDecision())
  {
    // std::cout << "object has no lateral decision";
    return false;
  }
  const auto &decision = obstacle.LateralDecision();
  if (!decision.has_nudge())
  {
    // std::cout << "only support nudge now";
    return true;
  }
  const auto &nudge = decision.nudge(); /////////////////////////////////////////////////////////////
  for (const SpeedPoint &veh_point : discretized_vehicle_location_)
  {
    double time = veh_point.t;
    TrajectoryPoint trajectory_point = obstacle.GetPointAtTime(time);
    Box2d obs_box = obstacle.GetBoundingBox(trajectory_point);
    // project obs_box on reference line
    std::vector<Vec2d> corners;
    obs_box.GetAllCorners(&corners);
    std::vector<SLPoint> sl_corners;

    for (const auto &corner_xy : corners)
    {
      SLPoint cur_point;
      if (!reference_line_.XYToSL(corner_xy, &cur_point))
      {
        // std::cout << "Fail to map xy point " << corner_xy.DebugString() << " to " << cur_point.ShortDebugString();
        return false;
      }
      // shift box base on buffer
      cur_point.set_l(cur_point.l + nudge.distance_l);
      sl_corners.push_back(std::move(cur_point));
    }

    for (uint32_t i = 0; i < sl_corners.size(); ++i)
    {
      SLPoint sl_first = sl_corners[i % sl_corners.size()];
      SLPoint sl_second = sl_corners[(i + 1) % sl_corners.size()];
      if (sl_first.s < sl_second.s)
      {
        std::swap(sl_first, sl_second);
      }

      std::pair<double, double> bound = MapLateralConstraint(
          sl_first, sl_second, nudge.type, veh_point.s - Config_.back_edge_to_center, veh_point.s + Config_.front_edge_to_center);

      // update bound map
      double s_resolution = std::fabs(veh_point.v * time_resolution_);
      double updated_start_s = init_frenet_point_.s + veh_point.s - s_resolution;
      double updated_end_s = init_frenet_point_.s + veh_point.s + s_resolution;
      if (updated_end_s > evaluated_s_.back() || updated_start_s < evaluated_s_.front())
      {
        continue;
      }
      std::pair<uint32_t, uint32_t> update_index_range = FindInterval(updated_start_s, updated_end_s);

      for (uint32_t j = update_index_range.first; j <= update_index_range.second; ++j)
      {
        dynamic_obstacle_bound_[j].first = std::max(bound.first, dynamic_obstacle_bound_[j].first);
        dynamic_obstacle_bound_[j].second = std::min(bound.second, dynamic_obstacle_bound_[j].second);
      }
    }
  }
  return true;
}

/*
MapStaticObstacleWithDecision()考虑了静态障碍物以及相应的横向避让措施对可行驶区域宽度的影响，
处理结果static_obstacle_bound_保存了evaluated_s_中各采样点处的横向可行驶范围
*/
bool QpFrenetFrame::MapStaticObstacleWithDecision(const Obstacle &obstacle)
{
  if (!obstacle.HasLateralDecision())
  {
    // std::cout << "obstacle has no lateral decision";
    return false;
  }
  const auto &decision = obstacle.LateralDecision();
  if (!decision.has_nudge())
  {
    // std::cout << "only support nudge decision now";
    return true;
  }
  if (!MapNudgePolygon(common::math::Polygon2d(obstacle.PerceptionBoundingBox()), decision.nudge(),
                       &static_obstacle_bound_))
  {
    // std::cout << "fail to map polygon with id " << obstacle.Id() << " in qp frenet frame";
    return false;
  }
  return true;
}
/*
处理静态障碍物的主要思路是将其轮廓端点映射到Frenet坐标系，结合nudge的方向计算障碍物占据的横向范围，
由MapNudgePolygon()和MapNudgeLine()实现。
个人理解：静态障碍物的nudge肯定都是NO_NUDGE吧
*/
bool QpFrenetFrame::MapNudgePolygon(const common::math::Polygon2d &polygon, const ObjectNudge &nudge,
                                    std::vector<std::pair<double, double>> *const bound_map)
{
  // std::cout << "ObjectNudge: " << nudge.TypeName() << "\n";
  std::vector<SLPoint> sl_corners;
  for (const auto &corner_xy : polygon.points())
  {
    SLPoint corner_sl;
    if (!reference_line_.XYToSL(corner_xy, &corner_sl))
    {
      // std::cout << "Fail to map xy point " << corner_xy.DebugString() << " to " << corner_sl.DebugString();
      return false;
    }
    // shift box based on buffer
    // nudge decision buffer:
    // --- position for left nudge
    // --- negative for right nudge
    corner_sl.set_l(corner_sl.l + nudge.distance_l);
    sl_corners.push_back(std::move(corner_sl));
  }

  const auto corner_size = sl_corners.size();
  for (uint32_t i = 0; i < corner_size; ++i)
  {
    if (!MapNudgeLine(sl_corners[i], sl_corners[(i + 1) % corner_size], nudge.type, bound_map))
    {
      // std::cout << "Map box line (sl) " << sl_corners[i].DebugString() << "->"
      // << sl_corners[(i + 1) % corner_size].DebugString();
      return false;
    }
  }
  return true;
}

// MapNudgePolygon()中循环调用MapNudgeLine()，将障碍物的bounding box对自车轨迹的横向约束计算，转换为计算bounding
// box的4条边对自车轨迹的横向约束。
bool QpFrenetFrame::MapNudgeLine(const SLPoint &start, const SLPoint &end, const ObjectNudge::Type nudge_type,
                                 std::vector<std::pair<double, double>> *const constraint)
{
  // DCHECK_NOTNULL(constraint);

  const SLPoint &near_point = (start.s < end.s ? start : end);
  const SLPoint &further_point = (start.s < end.s ? end : start);
  // impact_index表示evaluated_s_中受影响的区间范围，2个index可能是相等的
  std::pair<uint32_t, uint32_t> impact_index = FindInterval(near_point.s, further_point.s);

  // s轴超出轨迹纵向范围
  if (further_point.s < start_s_ - Config_.back_edge_to_center || near_point.s > end_s_ + Config_.front_edge_to_center)
  {
    return true;
  }

  const double distance = std::max(further_point.s - near_point.s, kMathEpsilon);
  const double adc_half_width = Config_.FLAGS_vehicle_width / 2;

  // DCHECK_GT(constraint->size(), impact_index.second);
  for (uint32_t i = impact_index.first; i <= impact_index.second; ++i)
  {
    double weight = std::fabs((evaluated_s_[i] - near_point.s)) / distance;
    weight = std::min(1.0, std::max(weight, 0.0));
    double boundary = near_point.l * (1 - weight) + further_point.l * weight;

    if (nudge_type == ObjectNudge::Type::LEFT_NUDGE) //障碍物的标签为向左微调
    {
      boundary += adc_half_width;
      // first是lower bound，second是upper bound
      // lower bound增大，即将自车可行驶区域向左移动，即left nudge
      //而upper bound不变，其初始化为INF
      (*constraint)[i].first = std::max(boundary, (*constraint)[i].first);
    }
    else //障碍物的标签为向右边微调，或者是NO_NUDGE，因为车是靠右行使，所以障碍物要向右，对自主车没啥影响
    {
      boundary -= adc_half_width;
      (*constraint)[i].second = std::min(boundary, (*constraint)[i].second);
    }
    //若可行驶区域宽度constraint太窄(<0.3m)，则收缩feasible_longitudinal_upper_bound_
    if ((*constraint)[i].second < (*constraint)[i].first + 0.3)
    {
      if (i > 0)
      {
        feasible_longitudinal_upper_bound_ =
            std::min(evaluated_s_[i - 1] - kEpsilonTol, feasible_longitudinal_upper_bound_);
      }
      else
      {
        feasible_longitudinal_upper_bound_ = start_s_;
        return true;
      }

      // std::cout << "current mapping constraint, sl point impact index "
      // << "near_point: " << near_point.DebugString() << "further_point: " << further_point.DebugString()
      //  << "impact_index: " << impact_index << "(*constraint)[" << i << "]" << (*constraint)[i];
      break;
    }
  }

  return true;
}
// MapLateralConstraint()用来计算障碍物bounding box的一条边对自车横向轨迹的约束
std::pair<double, double> QpFrenetFrame::MapLateralConstraint(const SLPoint &start, const SLPoint &end,
                                                              const ObjectNudge::Type nudge_type, const double s_start,
                                                              const double s_end)
{
  constexpr double inf = std::numeric_limits<double>::infinity();
  std::pair<double, double> result = std::make_pair(-inf, inf);

  if (start.s > s_end || end.s < s_start)
  {
    return result;
  }
  double s_front = std::max(start.s, s_start);
  double s_back = std::min(end.s, s_end);

  double weight_back = 0.0;
  double weight_front = 0.0;

  if (end.s - start.s > std::numeric_limits<double>::epsilon())
  {
    weight_back = (s_back - end.s) / (end.s - start.s);
    weight_front = (s_front - start.s) / (end.s - start.s);
  }

  SLPoint front = common::math::InterpolateUsingLinearApproximation(start, end, weight_front);
  SLPoint back = common::math::InterpolateUsingLinearApproximation(start, end, weight_back);

  if (nudge_type == ObjectNudge::Type::RIGHT_NUDGE)
  {
    result.second = std::min(front.l, back.l);
  }
  else
  {
    result.first = std::max(front.l, back.l);
  }
  return result;
}

std::pair<uint32_t, uint32_t> QpFrenetFrame::FindInterval(const double start, const double end) const
{
  double new_start = std::max(start - Config_.front_edge_to_center, evaluated_s_.front());
  double new_end = std::min(end + Config_.back_edge_to_center, evaluated_s_.back());

  uint32_t start_index = FindIndex(new_start);
  uint32_t end_index = FindIndex(new_end);
  if (end_index == evaluated_s_.size())
  {
    --end_index;
  }

  return std::make_pair(start_index, end_index);
}

//根据HDMap和道路参考线，计算自车轨迹的横向约束
// hdmap_bound_初始化为vector {evaluated_s_.size(), std::make_pair(-inf, inf)}
bool QpFrenetFrame::CalculateHDMapBound()
{
  const double adc_half_width = Config_.FLAGS_vehicle_width / 2.0;
  for (uint32_t i = 0; i < hdmap_bound_.size(); ++i)
  {
    double left_bound = 0.0;
    double right_bound = 0.0;
    // bool suc = reference_line_.GetLaneWidth(evaluated_s_[i], &left_bound, &right_bound);
    bool suc = false;
    if (!suc)
    {
      // AWARN << "Extracting lane width failed at s = " << evaluated_s_[i];
      right_bound = Config_.FLAGS_default_reference_line_width / 2;
      left_bound = Config_.FLAGS_default_reference_line_width / 2;
    }
    //按照右手坐标系，自车前方为s轴正方向，hdmap_bound_[i].first是右侧边界，second是左侧边界
    hdmap_bound_[i].first = -right_bound + adc_half_width;
    hdmap_bound_[i].second = left_bound - adc_half_width;

    //如果右边界>左边界，不合理，则缩短纵向s上限feasible_longitudinal_upper_bound_到当前考察点
    if (hdmap_bound_[i].first >= hdmap_bound_[i].second)
    {
      // std::cout << "HD Map bound at " << evaluated_s_[i] << " is infeasible (" << hdmap_bound_[i].first << ", "
      //<< hdmap_bound_[i].second << ") ";
      // std::cout << "left_bound: " << left_bound << ", right_bound: " << right_bound;

      feasible_longitudinal_upper_bound_ = std::min(evaluated_s_[i], feasible_longitudinal_upper_bound_);
      SLPoint sl;
      sl.set_s(evaluated_s_[i]);
      Vec2d xy;
      if (!reference_line_.SLToXY(sl, &xy))
      {
        // std::cout << "Fail to calculate HDMap bound at s: " << sl.s() << ", l: " << sl.l();
        return false;
      }
      // std::cout << "evaluated point x: " << std::fixed << xy.x() << " y: " << xy.y();
      break;
    }
  }
  return true;
}

/*
CalculateObstacleBound()的处理分为2部分：静态障碍物和动态障碍物。首先判断障碍物是否有LateralDecision，若没有，则忽略，
因为对path没有影响（这个阶段的path重点考虑横向运动）。LateralDecision主要是指横向的nudge
*/
bool QpFrenetFrame::CalculateObstacleBound(const std::vector<const Obstacle *> &obstacles)
{
  for (const auto ptr_obstacle : obstacles)
  {
    if (!ptr_obstacle->HasLateralDecision())
    {
      continue;
    }
    if (ptr_obstacle->IsStatic())
    {
      if (!MapStaticObstacleWithDecision(*ptr_obstacle))
      {
        // std::cout << "mapping obstacle with id [" << ptr_obstacle->Id() << "] failed in qp frenet frame.";
        return false;
      }
    }
    else
    {
      if (!MapDynamicObstacleWithDecision(*ptr_obstacle))
      {
        // std::cout << "mapping obstacle with id [" << ptr_obstacle->Id() << "] failed in qp frenet frame.";
        return false;
      }
    }
  }
  return true;
}

uint32_t QpFrenetFrame::FindIndex(const double s) const
{
  auto upper_bound = std::upper_bound(evaluated_s_.begin(), evaluated_s_.end(), s);
  return static_cast<uint32_t>(std::distance(evaluated_s_.begin(), upper_bound));
}
