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

#include <utility>
#include <vector>
#include "reference_line_info.h"
#include "path_data.h"
#include "speed_data.h"

#include "active_set_spline_1d_solver.h"
#include "st_graph_data.h"

class QpSplineStGraph
{
public:
    QpSplineStGraph(Spline1dSolver *spline_solver,
                    const bool is_change_lane, ReferenceLineInfo &reference_line_info);

    bool Search(const StGraphData &st_graph_data,
                const std::pair<double, double> &accel_bound,
                SpeedData *const speed_data);

private:
    void Init();

    // Add st graph constraint
    bool AddConstraint(const TrajectoryPoint &init_point,
                       const planning::SpeedLimit &speed_limit,
                       const std::vector<ST_Boundary> &boundaries,
                       const std::pair<double, double> &accel_bound);

    // Add objective function
    bool AddKernel(const std::vector<ST_Boundary> &boundaries,
                   const planning::SpeedLimit &speed_limit);

    // solve
    bool Solve();

    // extract upper lower bound for constraint;
    bool GetSConstraintByTime(
        const std::vector<ST_Boundary> &boundaries, const double time,
        const double total_path_s, double *const s_upper_bound,
        double *const s_lower_bound) const;

    // reference line kernel is a constant s line at s = 250m
    bool AddCruiseReferenceLineKernel(const double weight);

    // follow line kernel
    bool AddFollowReferenceLineKernel(
        const std::vector<ST_Boundary> &boundaries, const double weight);

    // yield line kernel
    bool AddYieldReferenceLineKernel(
        const std::vector<ST_Boundary> &boundaries, const double weight);

    const SpeedData GetHistorySpeed() const;
    bool EstimateSpeedUpperBound(
        const TrajectoryPoint &init_point, const planning::SpeedLimit &speed_limit,
        std::vector<double> *speed_upper_bound) const;

    bool AddDpStReferenceKernel(const double weight) const;

private:
    // solver
    Spline1dSolver *spline_solver_ = nullptr;

    // initial status
    TrajectoryPoint init_point_;

    // is change lane
    bool is_change_lane_ = false;

    // t knots resolution
    double t_knots_resolution_ = 0.0;

    // knots
    std::vector<double> t_knots_;

    // evaluated t resolution
    double t_evaluated_resolution_ = 0.0;

    // evaluated points
    std::vector<double> t_evaluated_;

    // reference line kernel
    std::vector<double> cruise_;

    // reference_line_info
    ReferenceLineInfo reference_line_info_;

    // reference st points from dp optimizer
    SpeedData reference_dp_speed_points_;
};