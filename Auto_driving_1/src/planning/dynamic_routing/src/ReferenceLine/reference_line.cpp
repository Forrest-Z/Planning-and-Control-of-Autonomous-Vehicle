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
  Modification: used only some functions
**/
#include "reference_line.h"
#include "math_utils.h"
#include "angle.h"

ReferenceLine::ReferenceLine(CubicSpline2D *csp, const std::vector<ReferencePoint> &reference_points,
                             const std::vector<double> &accumulated_s,
                             const std::pair<std::vector<double>, std::vector<double>> &reference_path)
    : csp_(csp), reference_points_(reference_points), accumulated_s_(accumulated_s), reference_path_(reference_path)
{
}

ReferencePoint ReferenceLine::GetReferencePoint(const double s) const
{
  auto comp = [](const ReferencePoint &point, const double &s)
  { return point.accumulated_s_ < s; };
  auto it_lower = std::lower_bound(reference_points_.begin(), reference_points_.end(), s, comp);
  if (it_lower == reference_points_.begin())
  {
    return reference_points_.front();
  }
  else if (it_lower == reference_points_.end())
  {
    return reference_points_.back();
  }
  return *it_lower;
}

ReferencePoint ReferenceLine::GetNearestReferencePoint(const Vec2d &xy) const
{
  double min_dist = std::numeric_limits<double>::max();
  size_t min_index = 0;
  for (size_t i = 0; i < reference_points_.size(); ++i)
  {
    const double distance = DistanceXY(xy, reference_points_[i]);
    if (distance < min_dist)
    {
      min_dist = distance;
      min_index = i;
    }
  }
  return reference_points_[min_index];
}

bool ReferenceLine::Segment(const Vec2d &point, const double look_backward, const double look_forward)
{
  SLPoint sl;
  if (!XYToSL(point, &sl))
  {
    // AERROR << "Failed to project point: " << point.DebugString();
    return false;
  }
  return Segment(sl.s, look_backward, look_forward);
}

bool ReferenceLine::Segment(const double s, const double look_backward, const double look_forward)
{
  // inclusive
  auto start_index = std::distance(accumulated_s_.begin(),
                                   std::lower_bound(accumulated_s_.begin(), accumulated_s_.end(), s - look_backward));

  // exclusive
  auto end_index = std::distance(accumulated_s_.begin(),
                                 std::upper_bound(accumulated_s_.begin(), accumulated_s_.end(), s + look_forward));

  if (end_index - start_index < 2)
  {
    // AERROR << "Too few reference points after shrinking.";
    return false;
  }
  return true;
}

FrenetFramePoint ReferenceLine::GetFrenetPoint(const PathPoint &path_point) const
{
  if (reference_points_.empty())
  {
    return FrenetFramePoint();
  }

  SLPoint sl_point;
  XYToSL(path_point, &sl_point);

  FrenetFramePoint frenet_frame_point;
  frenet_frame_point.set_s(sl_point.s);
  frenet_frame_point.set_l(sl_point.l);

  const double theta = path_point.theta;
  const double kappa = path_point.kappa;
  const double l = frenet_frame_point.d;

  // std::cout << "x:" << path_point.x << " "
  //           << "y:" << path_point.y << " "
  //           << "theta:" << theta << " "
  //           << "kappa:" << kappa << " "
  //           << "l:" << frenet_frame_point.d << "\n";

  ReferencePoint ref_point = GetReferencePoint(frenet_frame_point.s);

  const double theta_ref = ref_point.heading();
  const double kappa_ref = ref_point.kappa();
  const double dkappa_ref = ref_point.dkappa();

  const double dl = CartesianFrenetConverter::CalculateLateralDerivative(theta_ref, theta, l, kappa_ref);
  const double ddl = CartesianFrenetConverter::CalculateSecondOrderLateralDerivative(theta_ref, theta, kappa_ref, kappa,
                                                                                     dkappa_ref, l);
  frenet_frame_point.set_dl(dl);
  frenet_frame_point.set_ddl(ddl);
  return frenet_frame_point;
}

std::pair<std::array<double, 3>, std::array<double, 3>>
ReferenceLine::ToFrenetFrame(const TrajectoryPoint &traj_point) const
{
  //   ACHECK(!reference_points_.empty());

  SLPoint sl_point;
  XYToSL(traj_point.path_point(), &sl_point);

  std::array<double, 3> s_condition;
  std::array<double, 3> l_condition;
  ReferencePoint ref_point = GetReferencePoint(sl_point.s);
  CartesianFrenetConverter::cartesian_to_frenet(
      sl_point.s, ref_point.x_, ref_point.y_, ref_point.heading_, ref_point.kappa_, ref_point.dkappa_, traj_point.x,
      traj_point.y, traj_point.v, traj_point.a, traj_point.theta, traj_point.kappa, &s_condition, &l_condition);

  return std::make_pair(s_condition, l_condition);
}

ReferencePoint ReferenceLine::GetNearestReferencePoint(const double s) const
{
  // std::cout << accumulated_s_.size() << "\n";
  if (s < accumulated_s_.front() - 1e-2)
  {
    //  std::cout << "The requested s: " << s << " < 0.";
    return reference_points_.front();
  }
  if (s > accumulated_s_.back() + 1e-2)
  {
    // std::cout << "The requested s: " << s << " > reference line length: " << accumulated_s_.back();
    return reference_points_.back();
  }
  auto it_lower = std::lower_bound(accumulated_s_.begin(), accumulated_s_.end(), s);
  if (it_lower == accumulated_s_.begin())
  {
    return reference_points_.front();
  }
  auto index = std::distance(accumulated_s_.begin(), it_lower);
  if (std::fabs(accumulated_s_[index - 1] - s) < std::fabs(accumulated_s_[index] - s))
  {
    return reference_points_[index - 1];
  }
  return reference_points_[index];
}

size_t ReferenceLine::GetNearestReferenceIndex(const double s) const
{
  if (s < accumulated_s_.front() - 1e-2)
  {
    // AWARN << "The requested s: " << s << " < 0.";
    return 0;
  }
  if (s > accumulated_s_.back() + 1e-2)
  {
    // AWARN << "The requested s: " << s << " > reference line length " << accumulated_s_.back();
    return reference_points_.size() - 1;
  }
  auto it_lower = std::lower_bound(accumulated_s_.begin(), accumulated_s_.end(), s);
  return std::distance(accumulated_s_.begin(), it_lower);
}

std::vector<ReferencePoint> ReferenceLine::GetReferencePoints(double start_s, double end_s) const
{
  if (start_s < 0.0)
  {
    start_s = 0.0;
  }
  if (end_s > Length())
  {
    end_s = Length();
  }
  std::vector<ReferencePoint> ref_points;
  auto start_index = GetNearestReferenceIndex(start_s);
  auto end_index = GetNearestReferenceIndex(end_s);
  if (start_index < end_index)
  {
    ref_points.assign(reference_points_.begin() + start_index, reference_points_.begin() + end_index);
  }
  return ref_points;
}

bool ReferenceLine::SLToXY(const SLPoint &sl_point, Vec2d *const xy_point) const
{
  if (Length() < 2)
  {
    // AERROR << "The reference line has too few points.";
    return false;
  }
  const auto matched_point = GetReferencePoint(sl_point.s);
  const auto angle = Angle16::from_rad(matched_point.heading_);
  xy_point->set_x(matched_point.x_ - sin(angle) * sl_point.l);
  xy_point->set_y(matched_point.y_ + cos(angle) * sl_point.l);
  return true;
}

bool ReferenceLine::XYToSL(const Vec2d &xy_point, SLPoint *const sl_point) const
{
  if (Length() < 2)
  {
    // AERROR << "The reference line has too few points.";
    return false;
  }

  std::pair<double, double> initpoint(xy_point.x(), xy_point.y());
  int index = 0;
  std::pair<double, double> nearest = aux->Find_nearest_piont(initpoint, reference_path_, index);      //从参考线找最近的点
  double rtheta = reference_points_[index].heading();                                                  //角度
  double l = aux->cartesian_to_frenet(initpoint, nearest, rtheta);                                     // l
  double s = aux->find_s(csp_, nearest.first, nearest.second, accumulated_s_.front(), accumulated_s_); // s
  sl_point->set_s(s);
  sl_point->set_l(l);
  return true;
}

const std::vector<ReferencePoint> &ReferenceLine::reference_points() const
{
  return reference_points_;
}

bool ReferenceLine::GetLaneWidth(const double s, double *const lane_left_width, double *const lane_right_width) const
{
  if (Length() == 0)
  {
    return false;
  }

  //   if (!map_path_.GetLaneWidth(s, lane_left_width, lane_right_width))
  //   {
  //     return false;
  //   }
  return true;
}

bool ReferenceLine::GetRoadWidth(const double s, double *const road_left_width, double *const road_right_width) const
{
  //   if (Length() == 0)
  //   {
  //     return false;
  //   }
  //   return map_path_.GetRoadWidth(s, road_left_width, road_right_width);
  return true;
}

double ReferenceLine::GetDrivingWidth(const SL_Boundary &sl_boundary) const
{
  double lane_left_width = 0.0;
  double lane_right_width = 0.0;
  GetLaneWidth(sl_boundary.start_s_, &lane_left_width, &lane_right_width);

  double driving_width = std::max(lane_left_width - sl_boundary.end_l_, lane_right_width + sl_boundary.start_l_);
  driving_width = std::min(lane_left_width + lane_right_width, driving_width);
  //   std::cout << "Driving width [" << driving_width << "].";
  return driving_width;
}

bool ReferenceLine::IsOnLane(const Vec2d &vec2d_point) const
{
  SLPoint sl_point;
  if (!XYToSL(vec2d_point, &sl_point))
  {
    return false;
  }
  return IsOnLane(sl_point);
}

bool ReferenceLine::IsOnLane(const SLPoint &sl_point) const
{
  if (sl_point.s <= 0 || sl_point.s > Length())
  {
    return false;
  }
  double left_width = 0.0;
  double right_width = 0.0;

  if (!GetLaneWidth(sl_point.s, &left_width, &right_width))
  {
    return false;
  }

  return sl_point.l >= -right_width && sl_point.l <= left_width;
}

//是否在车道内
bool ReferenceLine::IsOnRoad(const Vec2d &vec2d_point) const
{
  return true;
}

// bool ReferenceLine::HasOverlap(const math::Box2d& box) const
// {
//    SL_Boundary sl_boundary;
//   if (!Get SL_Boundary(box, &sl_boundary))
//   {
//     // AERROR << "Failed to get sl boundary for box: " << box.DebugString();
//     return false;
//   }
//   if (sl_boundary.end_s() < 0 || sl_boundary.start_s() > Length())
//   {
//     return false;
//   }
//   if (sl_boundary.start_l() * sl_boundary.end_l() < 0)
//   {
//     return false;
//   }

//   double lane_left_width = 0.0;
//   double lane_right_width = 0.0;
//   const double mid_s = (sl_boundary.start_s() + sl_boundary.end_s()) / 2.0;
//   if (mid_s < 0 || mid_s > Length())
//   {
//     // ADEBUG << "ref_s is out of range: " << mid_s;
//     return false;
//   }
//   if (!map_path_.GetLaneWidth(mid_s, &lane_left_width, &lane_right_width))
//   {
//     // AERROR << "Failed to get width at s = " << mid_s;
//     return false;
//   }
//   if (sl_boundary.start_l() > 0)
//   {
//     return sl_boundary.start_l() < lane_left_width;
//   }
//   else
//   {
//     return sl_boundary.end_l() > -lane_right_width;
//   }
// }

TrajectoryPoint ReferenceLine::GetPointAtTime(const double relative_time, const SL_Boundary perception_obstacle_,
                                              const std::vector<TrajectoryPoint> points) const
{
  TrajectoryPoint ty;
  if (points.size() < 2)
  {
    TrajectoryPoint point;
    point.set_x(perception_obstacle_.centerpoint.position.x);
    point.set_y(perception_obstacle_.centerpoint.position.y);
    point.set_z(perception_obstacle_.centerpoint.position.z);
    point.set_theta(perception_obstacle_.obstacle_threa);
    point.set_s(0.0);
    point.set_kappa(0.0);
    point.set_dkappa(0.0);
    point.set_v(0.0);
    point.set_a(0.0);
    point.set_relative_time(0.0);
    return point;
  }
  else
  {
    auto comp = [](const TrajectoryPoint p, const double time)
    { return p.relative_time < time; };

    auto it_lower = std::lower_bound(points.begin(), points.end(), relative_time, comp);

    if (it_lower == points.begin())
    {
      return *points.begin();
    }
    else if (it_lower == points.end())
    {
      return *points.rbegin();
    }
    return common::math::InterpolateUsingLinearApproximation(*(it_lower - 1), *it_lower, relative_time);
  }
  return ty;
}

Box2d ReferenceLine::GetBoundingBox(TrajectoryPoint &point, SL_Boundary perception_obstacle_) const
{
  return Box2d({point.x, point.y}, point.theta, perception_obstacle_.obstacle_length,
               perception_obstacle_.obstacle_width);
}

double ReferenceLine::GetSpeedLimitFromS(const double s) const
{
  // 有速度限制段直接返回速度限制，该段的速度限制貌似是从高精地图获取的
  for (const auto &speed_limit : speed_limit_)
  {
    if (s >= speed_limit.start_s && s <= speed_limit.end_s)
    {
      return speed_limit.speed_limit;
    }
  }
  const auto &map_path_point = GetReferencePoint(s);

  double speed_limit = Config_.planning_upper_speed_limit;
  bool speed_limit_found = false;

  // 循环更新speed_limit的最小值
  // for (const auto &lane_waypoint : map_path_point.lane_waypoints())
  // {
  //   if (lane_waypoint.lane == nullptr)
  //   {
  //     // AWARN << "lane_waypoint.lane is nullptr.";
  //     continue;
  //   }
  //   speed_limit_found = true;
  //   speed_limit =
  //       std::fmin(lane_waypoint.lane->lane().speed_limit(), speed_limit);
  // }

  // 如果速度限制没有找到，则通过判断当前道路的类型，赋值不同道路类型默认的速度限制值
  // if (!speed_limit_found)
  // {
  //   // use default speed limit based on road_type
  //   speed_limit = Config_.FLAGS_default_city_road_speed_limit; // 15.67
  //   hdmap::Road::Type road_type = GetRoadType(s);
  //   if (road_type == hdmap::Road::HIGHWAY)
  //   {
  //     speed_limit = Config_.FLAGS_default_highway_speed_limit; // 29.06
  //   }
  // }

  return speed_limit;
}

void ReferenceLine::AddSpeedLimit(double start_s, double end_s,
                                  double speed_limit)
{
  std::vector<SpeedLimit> new_speed_limit;
  for (const auto &limit : speed_limit_)
  {
    if (start_s >= limit.end_s || end_s <= limit.start_s)
    {
      new_speed_limit.emplace_back(limit);
    }
    else
    {
      // start_s < speed_limit.end_s && end_s > speed_limit.start_s
      double min_speed = std::min(limit.speed_limit, speed_limit);
      if (start_s >= limit.start_s)
      {
        new_speed_limit.emplace_back(limit.start_s, start_s, min_speed);
        if (end_s <= limit.end_s)
        {
          new_speed_limit.emplace_back(start_s, end_s, min_speed);
          new_speed_limit.emplace_back(end_s, limit.end_s, limit.speed_limit);
        }
        else
        {
          new_speed_limit.emplace_back(start_s, limit.end_s, min_speed);
        }
      }
      else
      {
        new_speed_limit.emplace_back(start_s, limit.start_s, speed_limit);
        if (end_s <= limit.end_s)
        {
          new_speed_limit.emplace_back(limit.start_s, end_s, min_speed);
          new_speed_limit.emplace_back(end_s, limit.end_s, limit.speed_limit);
        }
        else
        {
          new_speed_limit.emplace_back(limit.start_s, limit.end_s, min_speed);
        }
      }
      start_s = limit.end_s;
      end_s = std::max(end_s, limit.end_s);
    }
  }
  speed_limit_.clear();
  if (end_s > start_s)
  {
    new_speed_limit.emplace_back(start_s, end_s, speed_limit);
  }
  for (const auto &limit : new_speed_limit)
  {
    if (limit.start_s < limit.end_s)
    {
      speed_limit_.emplace_back(limit);
    }
  }
  std::sort(speed_limit_.begin(), speed_limit_.end(),
            [](const SpeedLimit &a, const SpeedLimit &b)
            {
              if (a.start_s != b.start_s)
              {
                return a.start_s < b.start_s;
              }
              if (a.end_s != b.end_s)
              {
                return a.end_s < b.end_s;
              }
              return a.speed_limit < b.speed_limit;
            });
}

bool ReferenceLine::GetSLBoundary(const Box2d &box, SL_Boundary *const sl_boundary) const
{
  double start_s(std::numeric_limits<double>::max());
  double end_s(std::numeric_limits<double>::lowest());
  double start_l(std::numeric_limits<double>::max());
  double end_l(std::numeric_limits<double>::lowest());
  std::vector<Vec2d> corners;
  box.GetAllCorners(&corners);

  // The order must be counter-clockwise
  std::vector<SLPoint> sl_corners;
  for (const auto &point : corners)
  {
    SLPoint sl_point;
    if (!XYToSL(point, &sl_point))
    {
      return false;
    }
    sl_corners.push_back(std::move(sl_point));
  }

  for (size_t i = 0; i < corners.size(); ++i)
  {
    auto index0 = i;
    auto index1 = (i + 1) % corners.size();
    const auto &p0 = corners[index0];
    const auto &p1 = corners[index1];

    const auto p_mid = (p0 + p1) * 0.5;
    SLPoint sl_point_mid;
    if (!XYToSL(p_mid, &sl_point_mid))
    {
      return false;
    }

    Vec2d v0(sl_corners[index1].s - sl_corners[index0].s,
             sl_corners[index1].l - sl_corners[index0].l);

    Vec2d v1(sl_point_mid.s - sl_corners[index0].s,
             sl_point_mid.l - sl_corners[index0].l);

     sl_boundary->add_boundary_point(sl_corners[index0]);

    // sl_point is outside of polygon; add to the vertex list
    if (v0.CrossProd(v1) < 0.0)
    {
      sl_boundary->add_boundary_point(sl_point_mid);
    }
  }

  for (const auto &sl_point : sl_boundary->boundary_point())
  {
    start_s = std::fmin(start_s, sl_point.s);
    end_s = std::fmax(end_s, sl_point.s);
    start_l = std::fmin(start_l, sl_point.l);
    end_l = std::fmax(end_l, sl_point.l);
  }

  sl_boundary->start_s_ = start_s;
  sl_boundary->end_s_ = end_s;
  sl_boundary->start_l_ = start_l;
  sl_boundary->end_l_ = end_l;
  return true;
}
