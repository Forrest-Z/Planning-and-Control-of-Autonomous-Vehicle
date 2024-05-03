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
#include <memory>
#include <utility>
#include <vector>

#include "path_data.h"
#include "speed_data.h"

#include "piecewise_linear_generator.h"
#include "st_graph_data.h"

class QpPiecewiseStGraph
{
public:
    QpPiecewiseStGraph();

    bool Search(const StGraphData &st_graph_data,
                SpeedData *const speed_data,
                const std::pair<double, double> &accel_bound);

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

    // generate reference speed profile
    // bool ApplyReferenceSpeedProfile();
    bool AddCruiseReferenceLineKernel(const planning::SpeedLimit &speed_limit,
                                      const double weight);

    bool AddFollowReferenceLineKernel(
        const std::vector<ST_Boundary> &boundaries, const double weight);

    bool EstimateSpeedUpperBound(
        const TrajectoryPoint &init_point, const planning::SpeedLimit &speed_limit,
        std::vector<double> *speed_upper_bound) const;

private:
    // initial status
    TrajectoryPoint init_point_;

    // solver
    std::unique_ptr<PiecewiseLinearGenerator> generator_ = nullptr;

    // evaluated t resolution
    double t_evaluated_resolution_ = 0.0;

    // evaluated points
    std::vector<double> t_evaluated_;

    // reference line kernel
    std::vector<double> cruise_;
};
