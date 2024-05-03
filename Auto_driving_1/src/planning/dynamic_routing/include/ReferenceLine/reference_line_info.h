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

#include <limits>
#include <list>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include <ros/ros.h>
#include "vec2d.h"
#include "box2d.h"
#include "reference_line.h"
#include "Obstacle.h"
#include "path_points.h"
#include "FrenetPath.h"
#include "path_data.h"
#include "speed_data.h"
#include "path_boundary.h"
#include "PlanningTarget.h"

/**
 * @class ReferenceLineInfo
 * @brief ReferenceLineInfo holds all data for one reference line.
 */
class ReferenceLineInfo
{
public:
  enum class LaneType
  {
    LeftForward,
    LeftReverse,
    RightForward,
    RightReverse
  };
  ReferenceLineInfo() = default;

  ReferenceLineInfo(const InitialConditions &adc_planning_point, const ReferenceLine &reference_line);

  const ReferenceLine &reference_line() const;
  ReferenceLine *mutable_reference_line();

  void SetTrajectory(const DiscretizedTrajectory &trajectory);
  const DiscretizedTrajectory &trajectory() const;

  double Cost() const
  {
    return cost_;
  }
  void AddCost(double cost)
  {
    cost_ += cost;
  }
  void SetCost(double cost)
  {
    cost_ = cost;
  }
  double PriorityCost() const
  {
    return priority_cost_;
  }
  void SetPriorityCost(double cost)
  {
    priority_cost_ = cost;
  }

  const SpeedData &speed_data() const;
  void set_speed_data(SpeedData &speed_);

  /**
   * @brief check if current reference line is started from another reference
   *line info line. The method is to check if the start point of current
   *reference line is on previous reference line info.
   * @return returns true if current reference line starts on previous reference
   *line, otherwise false.
   **/
  bool IsStartFrom(const ReferenceLineInfo &previous_reference_line_info) const;

  // adjust trajectory if it starts from cur_vehicle postion rather planning
  // init point from upstream
  bool AdjustTrajectoryWhichStartsFromCurrentPos(const TrajectoryPoint &planning_start_point,
                                                 const std::vector<TrajectoryPoint> &trajectory,
                                                 DiscretizedTrajectory *adjusted_trajectory);

  std::string PathSpeedDebugString() const;

  /**
   * Check if the current reference line is a change lane reference line, i.e.,
   * ADC's current position is not on this reference line.
   */
  bool IsChangeLanePath() const;

  /**
   * Check if the current reference line is the neighbor of the vehicle
   * current position
   */
  bool IsNeighborLanePath() const;

  /**
   * Set if the vehicle can drive following this reference line
   * A planner need to set this value to true if the reference line is OK
   */
  void SetDrivable(bool drivable);
  bool IsDrivable() const;

  double OffsetToOtherReferenceLine() const
  {
    return offset_to_other_reference_line_;
  }
  void SetOffsetToOtherReferenceLine(const double offset)
  {
    offset_to_other_reference_line_ = offset;
  }

  const std::vector<PathBoundary> &GetCandidatePathBoundaries() const;

  void SetCandidatePathBoundaries(std::vector<PathBoundary> &&candidate_path_boundaries);

  const std::vector<PathData> &GetCandidatePathData() const;

  void SetCandidatePathData(std::vector<PathData> &&candidate_path_data);

  Obstacle *GetBlockingObstacle() const
  {
    return blocking_obstacle_;
  }
  void SetBlockingObstacle(const std::string &blocking_obstacle_id);

  bool is_path_lane_borrow() const
  {
    return is_path_lane_borrow_;
  }
  void set_is_path_lane_borrow(const bool is_path_lane_borrow)
  {
    is_path_lane_borrow_ = is_path_lane_borrow;
  }

  void set_is_on_reference_line()
  {
    is_on_reference_line_ = true;
  }

  uint32_t GetPriority() const
  {
    return reference_line_.GetPriority();
  }

  void SetPriority(uint32_t priority)
  {
    reference_line_.SetPriority(priority);
  }

  // different types of overlaps that can be handled by different scenarios.
  enum OverlapType
  {
    CLEAR_AREA = 1,
    CROSSWALK = 2,
    OBSTACLE = 3,
    PNC_JUNCTION = 4,
    SIGNAL = 5,
    STOP_SIGN = 6,
    YIELD_SIGN = 7,
  };

  void set_path_reusable(const bool path_reusable)
  {
    path_reusable_ = path_reusable;
  }

  bool path_reusable() const
  {
    return path_reusable_;
  }

private:
  void InitFirstOverlaps();

  bool CheckChangeLane() const;

private:
  ReferenceLine reference_line_;

  /**
   * @brief this is the number that measures the goodness of this reference
   * line. The lower the better.
   */
  double cost_ = 0.0;

  bool is_drivable_ = true;

  Obstacle *blocking_obstacle_;

  std::vector<PathBoundary> candidate_path_boundaries_;
  std::vector<PathData> candidate_path_data_;

  PathData path_data_;
  PathData fallback_path_data_;
  SpeedData speed_data_;

  DiscretizedTrajectory discretized_trajectory_;

  /**
   * @brief SL boundary of stitching point (starting point of plan trajectory)
   * relative to the reference line
   */
  SL_Boundary adc_sl_boundary_;

  bool is_on_reference_line_ = false;

  bool is_path_lane_borrow_ = false;

  double offset_to_other_reference_line_ = 0.0;

  double priority_cost_ = 0.0;

  PlanningTarget planning_target_;

  double cruise_speed_ = 0.0;

  bool path_reusable_ = false;
};
