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
#pragma once

#include <string>
#include <utility>
#include <vector>
#include <ros/ros.h>
#include "vec2d.h"
#include "box2d.h"
#include "reference_point.h"
#include "boundarys.h"
#include "path_points.h"
#include "CubicSpline2D.h"
#include "cartesian_frenet_conversion.h"
#include "auxiliary_function.h"
#include "trajectoryPoint.h"

//屏蔽掉的函数先不使用,后面可能会用
template <typename U, typename V>
double DistanceXY(const U &u, const V &v)
{
  return std::hypot(u.x() - v.x_, u.y() - v.y_);
}

class ReferenceLine
{
public:
  ReferenceLine() = default;
  explicit ReferenceLine(const ReferenceLine &reference_line) = default; //需要深拷贝，没写
  explicit ReferenceLine(CubicSpline2D *csp, const std::vector<ReferencePoint> &reference_points,
                         const std::vector<double> &accumulated_s,
                         const std::pair<std::vector<double>, std::vector<double>> &reference_path);
  ~ReferenceLine() = default;
  explicit ReferenceLine(const std::vector<ReferencePoint> &reference_points);

  /** Stitch current reference line with the other reference line
   * The stitching strategy is to use current reference points as much as
   * possible. The following two examples show two successful stitch cases.
   *
   * Example 1
   * this:   |--------A-----x-----B------|
   * other:                 |-----C------x--------D-------|
   * Result: |------A-----x-----B------x--------D-------|
   * In the above example, A-B is current reference line, and C-D is the other
   * reference line. If part B and part C matches, we update current reference
   * line to A-B-D.
   *
   * Example 2
   * this:                  |-----A------x--------B-------|
   * other:  |--------C-----x-----D------|
   * Result: |--------C-----x-----A------x--------B-------|
   * In the above example, A-B is current reference line, and C-D is the other
   * reference line. If part A and part D matches, we update current reference
   * line to C-A-B.
   *
   * @return false if these two reference line cannot be stitched
   */
  bool Stitch(const ReferenceLine &other);

  bool Segment(const Vec2d &point, const double distance_backward, const double distance_forward);

  bool Segment(const double s, const double distance_backward, const double distance_forward);

  const std::vector<ReferencePoint> &reference_points() const;

  ReferencePoint GetReferencePoint(const double s) const;

  FrenetFramePoint GetFrenetPoint(const PathPoint &path_point) const;

  std::pair<std::array<double, 3>, std::array<double, 3>> ToFrenetFrame(const TrajectoryPoint &traj_point) const;

  std::vector<ReferencePoint> GetReferencePoints(double start_s, double end_s) const;

  size_t GetNearestReferenceIndex(const double s) const;

  ReferencePoint GetNearestReferencePoint(const Vec2d &xy) const;

  double GetSpeedLimitFromS(const double s) const;

  void AddSpeedLimit(double start_s, double end_s, double speed_limit);
  //   std::vector<hdmap::LaneSegment> GetLaneSegments(const double start_s, const double end_s) const;

  ReferencePoint GetNearestReferencePoint(const double s) const;

  ReferencePoint GetReferencePoint(const double x, const double y) const;

  bool SLToXY(const SLPoint &sl_point, Vec2d *const xy_point) const;
  bool XYToSL(const Vec2d &xy_point, SLPoint *const sl_point) const;
  template <class XYPoint>
  bool XYToSL(const XYPoint &xy, SLPoint *const sl_point) const
  {
    return XYToSL(Vec2d(xy.x, xy.y), sl_point);
  }
  TrajectoryPoint GetPointAtTime(const double relative_time, const SL_Boundary perception_obstacle_,
                                 const std::vector<TrajectoryPoint> points) const;

  Box2d GetBoundingBox(TrajectoryPoint &point, SL_Boundary perception_obstacle_) const;

  bool GetLaneWidth(const double s, double *const lane_left_width, double *const lane_right_width) const;

  bool GetOffsetToMap(const double s, double *l_offset) const;

  bool GetRoadWidth(const double s, double *const road_left_width, double *const road_right_width) const;

  //   hdmap::Road::Type GetRoadType(const double s) const;

  //   void GetLaneFromS(const double s, std::vector<hdmap::LaneInfoConstPtr>* lanes) const;

  double GetDrivingWidth(const SL_Boundary &sl_boundary) const;

  /**
   * @brief: check if a box/point is on lane along reference line
   */
  bool IsOnLane(const SLPoint &sl_point) const;
  bool IsOnLane(const Vec2d &vec2d_point) const;
  bool IsOnRoad(const Vec2d &vec2d_point) const;
  template <class XYPoint>
  bool IsOnLane(const XYPoint &xy) const
  {
    return IsOnLane(Vec2d(xy.x, xy.y));
  }

  /**
   * @brief Check if a box is blocking the road surface. The criteria is to
   * check whether the remaining space on the road surface is larger than the
   * provided gap space.
   * @param boxed the provided box
   * @param gap check the gap of the space
   * @return true if the box blocks the road.
   */
  bool IsBlockRoad(const Box2d &box2d, double gap) const;

  /**
   * @brief check if any part of the box has overlap with the road.
   */
  // bool HasOverlap(const Box2d& box) const;

  double Length() const
  {
    return accumulated_s_.size();
  }

  uint32_t GetPriority() const
  {
    return priority_;
  }
  void SetPriority(uint32_t priority)
  {
    priority_ = priority;
  }
  int test()
  {
    return reference_points_.size();
  }
  const bool is_valid_path_reference() const
  {
    return reference_points_.size() > 2;
  }

  std::vector<ReferencePoint> path_reference() const
  {
    return reference_points_;
  }

  bool GetSLBoundary(const Box2d& box, SL_Boundary* const sl_boundary) const;
private:
  /**
   * @brief Linearly interpolate p0 and p1 by s0 and s1.
   * The input has to satisfy condition: s0 <= s <= s1
   * p0 and p1 must have lane_waypoint.
   * Note: it requires p0 and p1 are on the same lane, adjacent lanes, or
   * parallel neighboring lanes. Otherwise the interpolated result may not
   * valid.
   * @param p0 the first anchor point for interpolation.
   * @param s0 the longitutial distance (s) of p0 on current reference line.
   * s0 <= s && s0 <= s1
   * @param p1 the second anchor point for interpolation
   * @param s1 the longitutial distance (s) of p1 on current reference line.
   * s1
   * @param s identifies the middle point that is going to be
   * interpolated.
   * s >= s0 && s <= s1
   * @return The interpolated ReferencePoint.
   */
  static ReferencePoint Interpolate(const ReferencePoint &p0, const double s0, const ReferencePoint &p1,
                                    const double s1, const double s);
  //   ReferencePoint InterpolateWithMatchedIndex(const ReferencePoint& p0, const double s0, const ReferencePoint& p1,
  //                                              const double s1, const InterpolatedIndex& index) const;

private:
  struct SpeedLimit
  {
    double start_s = 0.0;
    double end_s = 0.0;
    double speed_limit = 0.0; // unit m/s
    SpeedLimit() = default;
    SpeedLimit(double _start_s, double _end_s, double _speed_limit)
        : start_s(_start_s), end_s(_end_s), speed_limit(_speed_limit)
    {
    }
  };
  /**
   * This speed limit overrides the lane speed limit
   **/
  std::vector<SpeedLimit> speed_limit_;
  std::vector<ReferencePoint> reference_points_;
  CubicSpline2D *csp_;
  std::vector<double> accumulated_s_;
  std::pair<std::vector<double>, std::vector<double>> reference_path_;
  std::shared_ptr<Auxiliary> aux;
  //   hdmap::Path map_path_;
  uint32_t priority_ = 0;
};
