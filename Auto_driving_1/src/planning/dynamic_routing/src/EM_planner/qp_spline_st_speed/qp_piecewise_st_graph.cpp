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
#include "qp_piecewise_st_graph.h"

#include <algorithm>
#include <limits>
#include <string>

QpPiecewiseStGraph::QpPiecewiseStGraph()
    : t_evaluated_resolution_(Config_.total_time / Config_.number_of_evaluated_graph_t)
{
    Init();
}

void QpPiecewiseStGraph::Init()
{
    // init evaluated t positions
    double curr_t = t_evaluated_resolution_;
    t_evaluated_.resize(Config_.number_of_evaluated_graph_t);
    for (auto &t : t_evaluated_)
    {
        t = curr_t;
        curr_t += t_evaluated_resolution_;
    }
}

bool QpPiecewiseStGraph::Search(
    const StGraphData &st_graph_data, SpeedData *const speed_data,
    const std::pair<double, double> &accel_bound)
{
    cruise_.clear();

    init_point_ = st_graph_data.init_point();
    // ADEBUG << "Init point:" << init_point_.DebugString();

    // reset piecewise linear generator
    generator_.reset(new PiecewiseLinearGenerator(
        Config_.number_of_evaluated_graph_t,
        t_evaluated_resolution_));

    if (!AddConstraint(st_graph_data.init_point(), st_graph_data.speed_limit(),
                       st_graph_data.st_boundaries(), accel_bound))
    {
        const std::string msg = "Add constraint failed!";
        // //std::cout << msg;
        return false;
    }

    if (!AddKernel(st_graph_data.st_boundaries(), st_graph_data.speed_limit()))
    {
        const std::string msg = "Add kernel failed!";
        //std::cout << msg;
        return false;
    }

    if (!Solve())
    {
        const std::string msg = "Solve qp problem failed!";
        //std::cout << msg;
        return false;
    }

    // extract output
    speed_data->clear();
    const auto &res = generator_->params();
    speed_data->AppendSpeedPoint(0.0, 0.0, init_point_.v, init_point_.a, 0.0);

    double v = 0.0;
    double a = 0.0;

    double time = t_evaluated_resolution_;
    double dt = t_evaluated_resolution_;

    for (int i = 0; i < res.rows(); ++i, time += t_evaluated_resolution_)
    {
        double s = res(i, 0);
        if (i == 0)
        {
            v = s / dt;
            a = (v - init_point_.v) / dt;
        }
        else
        {
            const double curr_v = (s - res(i - 1, 0)) / dt;
            a = (curr_v - v) / dt;
            v = curr_v;
        }
        speed_data->AppendSpeedPoint(s, time, v, a, 0.0);
    }
    return true;
}

bool QpPiecewiseStGraph::AddConstraint(
    const TrajectoryPoint &init_point, const planning::SpeedLimit &speed_limit,
    const std::vector<ST_Boundary> &boundaries,
    const std::pair<double, double> &accel_bound)
{
    auto *constraint = generator_->mutable_constraint();
    // position, velocity, acceleration

    // monotone constraint
    if (!constraint->AddMonotoneInequalityConstraint())
    {
        const std::string msg = "add monotone inequality constraint failed!";
        //std::cout << msg;
        return false;
    }

    std::vector<uint32_t> index_list(t_evaluated_.size());

    // boundary constraint
    std::vector<double> s_upper_bound(t_evaluated_.size());
    std::vector<double> s_lower_bound(t_evaluated_.size());

    for (uint32_t i = 0; i < t_evaluated_.size(); ++i)
    {
        index_list[i] = i;
        const double curr_t = t_evaluated_[i];
        double lower_s = 0.0;
        double upper_s = 0.0;
        GetSConstraintByTime(boundaries, curr_t, Config_.total_path_length, &upper_s, &lower_s);
        s_upper_bound[i] = upper_s;
        s_lower_bound[i] = lower_s;
        // ADEBUG << "Add constraint by time: " << curr_t << " upper_s: " << upper_s
        //        << " lower_s: " << lower_s;
    }
    if (!constraint->AddBoundary(index_list, s_lower_bound, s_upper_bound))
    {
        const std::string msg = "Fail to apply distance constraints.";
        //std::cout << msg;
        return false;
    }

    // speed constraint
    std::vector<double> speed_upper_bound;
    if (!EstimateSpeedUpperBound(init_point, speed_limit, &speed_upper_bound))
    {
        std::string msg = "Fail to estimate speed upper constraints.";
        //std::cout << msg;
        return false;
    }

    std::vector<double> speed_lower_bound(t_evaluated_.size(), 0.0);

    if (!constraint->AddDerivativeBoundary(index_list, speed_lower_bound,
                                           speed_upper_bound))
    {
        const std::string msg = "Fail to apply speed constraints.";
        //std::cout << msg;
        return false;
    }
    // for (size_t i = 0; i < t_evaluated_.size(); ++i)
    // {
    //     std::cout << "t_evaluated_: " << t_evaluated_[i]
    //               << "; speed_lower_bound: " << speed_lower_bound[i]
    //               << "; speed_upper_bound: " << speed_upper_bound[i];
    // }

    // acceleration constraint
    std::vector<double> accel_lower_bound(t_evaluated_.size(), accel_bound.first);
    std::vector<double> accel_upper_bound(t_evaluated_.size(),
                                          accel_bound.second);

    if (!constraint->AddSecondDerivativeBoundary(
            init_point.v, index_list, accel_lower_bound, accel_upper_bound))
    {
        const std::string msg = "Fail to apply acceleration constraints.";
        return false;
    }
    // for (size_t i = 0; i < t_evaluated_.size(); ++i)
    // {
    //     std::cout << "t_evaluated_: " << t_evaluated_[i]
    //               << "; accel_lower_bound: " << accel_lower_bound[i]
    //               << "; accel_upper_bound: " << accel_upper_bound[i];
    // }

    return true;
}

bool QpPiecewiseStGraph::AddKernel(
    const std::vector<ST_Boundary> &boundaries,
    const planning::SpeedLimit &speed_limit)
{
    auto *kernel = generator_->mutable_kernel();
    // DCHECK_NOTNULL(kernel);

    if (Config_.accel_kernel_weight > 0)
    {
        kernel->AddSecondOrderDerivativeMatrix(
            init_point_.v,
            Config_.accel_kernel_weight);
    }

    if (Config_.jerk_kernel_weight > 0)
    {
        kernel->AddThirdOrderDerivativeMatrix(
            init_point_.v, init_point_.a,
            Config_.jerk_kernel_weight);
    }

    if (!AddCruiseReferenceLineKernel(speed_limit, Config_.cruise_weight))
    {
        return false;
    }

    if (!AddFollowReferenceLineKernel(boundaries, Config_.follow_weight))
    {
        return false;
    }
    kernel->AddRegularization(Config_.regularization_weight);
    return true;
}

bool QpPiecewiseStGraph::AddCruiseReferenceLineKernel(
    const planning::SpeedLimit &speed_limit, const double weight)
{
    auto *ref_kernel = generator_->mutable_kernel();
    if (speed_limit.speed_limit_points().size() == 0)
    {
        std::string msg = "Fail to apply_kernel due to empty speed limits.";
        //std::cout << msg;
        return false;
    }
    std::vector<uint32_t> index_list(t_evaluated_.size());
    cruise_.resize(t_evaluated_.size());

    for (uint32_t i = 0; i < t_evaluated_.size(); ++i)
    {
        index_list[i] = i;
        cruise_[i] = Config_.total_path_length;
    }

    // for (size_t i = 0; i < t_evaluated_.size(); ++i)
    // {
    //     std::cout << "Cruise Ref S: " << cruise_[i]
    //            << " Relative time: " << t_evaluated_[i] << std::endl;
    // }

    if (t_evaluated_.size() > 0)
    {
        ref_kernel->AddReferenceLineKernelMatrix(
            index_list, cruise_, weight * static_cast<double>(t_evaluated_.size()) / Config_.total_time);
    }

    return true;
}

bool QpPiecewiseStGraph::AddFollowReferenceLineKernel(
    const std::vector<ST_Boundary> &boundaries, const double weight)
{
    auto *follow_kernel = generator_->mutable_kernel();
    std::vector<double> ref_s;
    std::vector<double> filtered_evaluate_t;
    std::vector<uint32_t> index_list;
    for (size_t i = 0; i < t_evaluated_.size(); ++i)
    {
        const double curr_t = t_evaluated_[i];
        double s_min = std::numeric_limits<double>::infinity();
        bool success = false;
        for (const auto &boundary : boundaries)
        {
            if (boundary.boundary_type() != ST_Boundary::BoundaryType::FOLLOW)
            {
                continue;
            }
            if (curr_t < boundary.min_t_ || curr_t > boundary.max_t_)
            {
                continue;
            }
            double s_upper = 0.0;
            double s_lower = 0.0;
            if (boundary.GetUnblockSRange(curr_t, &s_upper, &s_lower))
            {
                success = true;
                s_min = std::min(s_min, s_upper - boundary.characteristic_length() - Config_.follow_drag_distance);
            }
        }
        if (success && s_min < cruise_[i])
        {
            filtered_evaluate_t.push_back(curr_t);
            ref_s.push_back(s_min);
            index_list.push_back(static_cast<uint32_t>(i));
        }
    }
    // DCHECK_EQ(filtered_evaluate_t.size(), ref_s.size());

    if (!ref_s.empty())
    {
        follow_kernel->AddReferenceLineKernelMatrix(
            index_list, ref_s, weight * static_cast<double>(t_evaluated_.size()) / Config_.total_time);
    }

    // for (size_t i = 0; i < filtered_evaluate_t.size(); ++i)
    // {
    //     std::cout << "Follow Ref S: " << ref_s[i]
    //               << " Relative time: " << filtered_evaluate_t[i] << std::endl;
    // }
    return true;
}

bool QpPiecewiseStGraph::GetSConstraintByTime(
    const std::vector<ST_Boundary> &boundaries, const double time,
    const double total_path_s, double *const s_upper_bound,
    double *const s_lower_bound) const
{
    *s_upper_bound = total_path_s;

    for (ST_Boundary boundary : boundaries)
    {
        double s_upper = 0.0;
        double s_lower = 0.0;

        if (!boundary.GetUnblockSRange(time, &s_upper, &s_lower))
        {
            continue;
        }

        if (boundary.boundary_type() == ST_Boundary::BoundaryType::STOP ||
            boundary.boundary_type() == ST_Boundary::BoundaryType::FOLLOW ||
            boundary.boundary_type() == ST_Boundary::BoundaryType::YIELD)
        {
            *s_upper_bound = std::fmin(*s_upper_bound, s_upper);
        }
        else
        {
            // DCHECK(boundary->boundary_type() == ST_Boundary::BoundaryType::OVERTAKE);
            *s_lower_bound = std::fmax(*s_lower_bound, s_lower);
        }
    }

    return true;
}

bool QpPiecewiseStGraph::EstimateSpeedUpperBound(
    const TrajectoryPoint &init_point, const planning::SpeedLimit &speed_limit,
    std::vector<double> *speed_upper_bound) const
{
    // TODO(Lianliang): define a QpStGraph class and move this function in it.
    // DCHECK_NOTNULL(speed_upper_bound);

    speed_upper_bound->clear();

    // use v to estimate position: not accurate, but feasible in cyclic
    // processing. We can do the following process multiple times and use
    // previous cycle's results for better estimation.
    const double v = init_point.v;

    if (static_cast<double>(t_evaluated_.size() +
                            speed_limit.speed_limit_points().size()) <
        static_cast<double>(t_evaluated_.size()) *
            std::log(
                static_cast<double>(speed_limit.speed_limit_points().size())))
    {
        uint32_t i = 0;
        uint32_t j = 0;
        const double kDistanceEpsilon = 1e-6;
        while (i < t_evaluated_.size() &&
               j + 1 < speed_limit.speed_limit_points().size())
        {
            const double distance = v * t_evaluated_[i];
            if (fabs(distance - speed_limit.speed_limit_points()[j].first) <
                kDistanceEpsilon)
            {
                speed_upper_bound->push_back(
                    speed_limit.speed_limit_points()[j].second);
                ++i;
                // ADEBUG << "speed upper bound:" << speed_upper_bound->back();
            }
            else if (distance < speed_limit.speed_limit_points()[j].first)
            {
                ++i;
            }
            else if (distance <= speed_limit.speed_limit_points()[j + 1].first)
            {
                speed_upper_bound->push_back(speed_limit.GetSpeedLimitByS(distance));
                // ADEBUG << "speed upper bound:" << speed_upper_bound->back();
                ++i;
            }
            else
            {
                ++j;
            }
        }

        for (size_t k = speed_upper_bound->size(); k < t_evaluated_.size(); ++k)
        {
            speed_upper_bound->push_back(Config_.planning_upper_speed_limit);
            // ADEBUG << "speed upper bound:" << speed_upper_bound->back();
        }
    }
    else
    {
        auto cmp = [](const std::pair<double, double> &p1, const double s)
        {
            return p1.first < s;
        };

        const auto &speed_limit_points = speed_limit.speed_limit_points();
        for (const double t : t_evaluated_)
        {
            const double s = v * t;

            // NOTICE: we are using binary search here based on two assumptions:
            // (1) The s in speed_limit_points increase monotonically.
            // (2) The evaluated_t_.size() << number of speed_limit_points.size()
            //
            // If either of the two assumption is failed, a new algorithm must be
            // used
            // to replace the binary search.

            const auto &it = std::lower_bound(speed_limit_points.begin(),
                                              speed_limit_points.end(), s, cmp);
            if (it != speed_limit_points.end())
            {
                speed_upper_bound->push_back(it->second);
            }
            else
            {
                speed_upper_bound->push_back(speed_limit_points.back().second);
            }
        }
    }

    const double kTimeBuffer = 2.0;
    const double kSpeedBuffer = 0.1;
    for (uint32_t k = 0; k < t_evaluated_.size() && t_evaluated_[k] < kTimeBuffer;
         ++k)
    {
        speed_upper_bound->at(k) =
            std::fmax(init_point_.v + kSpeedBuffer, speed_upper_bound->at(k));
    }

    return true;
}

bool QpPiecewiseStGraph::Solve()
{
    return generator_->Solve()
               ? true
               : false;
}