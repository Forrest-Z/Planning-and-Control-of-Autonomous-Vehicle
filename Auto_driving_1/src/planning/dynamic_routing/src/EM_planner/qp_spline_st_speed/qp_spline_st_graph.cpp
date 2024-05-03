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
#include "qp_spline_st_graph.h"
#include <algorithm>
#include <limits>
#include <string>
#include <memory>

QpSplineStGraph::QpSplineStGraph(Spline1dSolver *spline_solver,
                                 const bool is_change_lane, ReferenceLineInfo &reference_line_info)
    : spline_solver_(spline_solver),
      is_change_lane_(is_change_lane),
      t_knots_resolution_(
          Config_.total_time /
          Config_.number_of_discrete_graph_t),
      reference_line_info_(reference_line_info)
{
    Init();
}
// 速度规划器中设置了3段，所以应该是4个knots，存储了4个knots(t_knots_.size(): 4)， 每段同样使用一个5次多项式去拟合。
void QpSplineStGraph::Init()
{
    // init knots
    double curr_t = 0.0;
    uint32_t num_spline = Config_.number_of_discrete_graph_t - 1; //3
    for (uint32_t i = 0; i <= num_spline; ++i)
    {
        t_knots_.push_back(curr_t); // 0，1.75， 3.5，5.25
        curr_t += t_knots_resolution_;
    }

    uint32_t num_evaluated_t = 10 * num_spline + 1; // 10 * 4 +1

    // init evaluated t positions
    curr_t = 0;
    t_evaluated_resolution_ = Config_.total_time / (num_evaluated_t - 1); // 7/41
    for (uint32_t i = 0; i < num_evaluated_t; ++i)
    {
        t_evaluated_.push_back(curr_t);
        curr_t += t_evaluated_resolution_;
    }
}

bool QpSplineStGraph::Search(const StGraphData &st_graph_data,
                             const std::pair<double, double> &accel_bound,
                             SpeedData *const speed_data)
{
    constexpr double kBounadryEpsilon = 1e-2;
    //障碍物处理，如果要碰撞，给0
    for (const auto &boundary : st_graph_data.st_boundaries())
    {
        if (boundary.IsPointInBoundary({0.0, 0.0}) ||
            (std::fabs(boundary.min_t_) < kBounadryEpsilon &&
             std::fabs(boundary.min_s_) < kBounadryEpsilon))
        {
            speed_data->clear();
            const double t_output_resolution = Config_.FLAGS_trajectory_time_min_interval;
            double time = 0.0;
            while (time < Config_.total_time + t_output_resolution)
            {
                speed_data->AppendSpeedPoint(0.0, time, 0.0, 0.0, 0.0);
                time += t_output_resolution;
            }
            return true;
        }
    }

    cruise_.clear();
    reference_dp_speed_points_ = *speed_data;

    init_point_ = st_graph_data.init_point();

    // reset spline generator
    spline_solver_->Reset(t_knots_, Config_.spline_order);

    // for (int i = 0; i < st_graph_data.speed_limit().speed_limit_points().size(); ++i)
    // {
    //     std::cout << "s:" << st_graph_data.speed_limit().speed_limit_points()[i].first << " "
    //               << "v:" << st_graph_data.speed_limit().speed_limit_points()[i].second << "\n";
    // }

    if (!AddConstraint(st_graph_data.init_point(), st_graph_data.speed_limit(),
                       st_graph_data.st_boundaries(), accel_bound))
    {
        const std::string msg = "Add constraint failed!";
        std::cout << msg;
        return false;
    }

    if (!AddKernel(st_graph_data.st_boundaries(), st_graph_data.speed_limit()))
    {
        const std::string msg = "Add kernel failed!";
        std::cout << msg;
        return false;
    }

    auto start = std::chrono::system_clock::now();
    if (!Solve())
    {
        const std::string msg = "Solve qp problem failed!";
        std::cout << msg;
        return false;
    }
    auto end = std::chrono::system_clock::now();
    // std::chrono::duration<double> diff = end_time - start_time;
    // std::cout << "Path Optimizer used time: " << diff.count() * 1000 << " ms.";

    // extract output
    speed_data->clear(); //先清空原来的数据
    const Spline1d &spline = spline_solver_->spline();

    const double t_output_resolution = Config_.FLAGS_trajectory_time_min_interval;
    double time = 0.0;
    while (time < Config_.total_time + t_output_resolution)
    {
        double s = spline(time);
        double v = std::max(0.0, spline.Derivative(time));
        double a = spline.SecondOrderDerivative(time);
        double da = spline.ThirdOrderDerivative(time);
        speed_data->AppendSpeedPoint(s, time, v, a, da);
        time += t_output_resolution;
    }

    return true;
}

bool QpSplineStGraph::AddConstraint(
    const TrajectoryPoint &init_point, const planning::SpeedLimit &speed_limit,
    const std::vector<ST_Boundary> &boundaries,
    const std::pair<double, double> &accel_bound)
{
    Spline1dConstraint *constraint = spline_solver_->mutable_spline_constraint();
    //等式约束1：多项式函数必须经过原点，也就是t=0，delta_s=0(无人车当前位置)
    if (!constraint->AddPointConstraint(0.0, 0.0))
    {
        const std::string msg = "add st start point constraint failed";
        //std::cout << msg;
        return false;
    }
    //等式约束2：原点处的斜率，必须等于规划起始点速度
    if (!constraint->AddPointDerivativeConstraint(0.0, init_point_.v))
    {
        const std::string msg = "add st start point velocity constraint failed!";
        //std::cout << msg;
        return false;
    }
    //不等式约束3：函数单调性约束
    // monotone constraint
    if (!constraint->AddMonotoneInequalityConstraint(t_evaluated_))
    {
        const std::string msg = "add monotone inequality constraint failed!";
        //std::cout << msg;
        return false;
    }

    /*
    等式约束4：段拟合函数间平滑约束 
     现共有4段拟合函数，和参考线平滑中完全相同。相邻两个段拟合函数之间，必须保证：

        前一个函数终点和后一个函数起点函数值相同，即连续对应相对时间t时刻，两段上无人车的位置必须是相同的
        前一个函数终点和后一个函数一阶导相同，对应相对时间t时刻，两段上无人车的速度必须是相同的
        前一个函数终点和后一个函数二阶导相同，对应相对时间t时刻，两段上无人车的加速度必须是相同的
        前一个函数终点和后一个函数三阶导相同，对应相对时间t时刻，两段上无人车的加速度抖动必须是相同的
 
    */
    // smoothness constraint
    if (!constraint->AddThirdDerivativeSmoothConstraint())
    {
        const std::string msg = "add smoothness joint constraint failed!";
        //std::cout << msg;
        return false;
    }

    //不等式约束5：边界框约束
    //实现已经采样了t_evaluated_，从[0,8]秒区间内均匀采样了41个点，每两个点之间的距离为8/40。所以边界约束，
    //只要计算所有障碍物在每个采样点内的st边界框，满足规划点也就是delta_s=f(t)在st边界框外面即可。
    // boundary constraint
    std::vector<double> s_upper_bound;
    std::vector<double> s_lower_bound;

    for (const double curr_t : t_evaluated_)
    {
        double lower_s = 0.0;
        double upper_s = 0.0;
        GetSConstraintByTime(boundaries, curr_t,
                             Config_.total_path_length, &upper_s,
                             &lower_s);
        s_upper_bound.push_back(upper_s);
        s_lower_bound.push_back(lower_s);
        //std::cout << "Add constraint by time: " << curr_t << " upper_s: " << upper_s
        // << " lower_s: " << lower_s;
    }

    // DCHECK_EQ(t_evaluated_.size(), s_lower_bound.size());
    // DCHECK_EQ(t_evaluated_.size(), s_upper_bound.size());
    if (!constraint->AddBoundary(t_evaluated_, s_lower_bound, s_upper_bound))
    {
        const std::string msg = "Fail to apply distance constraints.";
        //std::cout << msg;
        return false;
    }

    //不等式约束6：限速约束
    // speed constraint
    std::vector<double> speed_upper_bound;
    if (!EstimateSpeedUpperBound(init_point, speed_limit, &speed_upper_bound))
    {
        std::string msg = "Fail to estimate speed upper constraints.";
        //std::cout << msg;
        return false;
    }
    // for (int i = 0; i < speed_upper_bound.size(); ++i)
    // {
    //     std::cout << speed_upper_bound[i] << "\n";
    // }
    // std::cout << "11111111111111111111111111"
    //           << "\n";

    std::vector<double> speed_lower_bound(t_evaluated_.size(), 0.0);

    // DCHECK_EQ(t_evaluated_.size(), speed_upper_bound.size());
    // DCHECK_EQ(t_evaluated_.size(), speed_lower_bound.size());

    if (!constraint->AddDerivativeBoundary(t_evaluated_, speed_lower_bound,
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
    //               << "; speed_upper_bound: " << speed_upper_bound[i] << "\n";
    // }
    //不等式约束7：加速度约束
    // acceleration constraint
    std::vector<double> accel_lower_bound(t_evaluated_.size(), accel_bound.first);
    std::vector<double> accel_upper_bound(t_evaluated_.size(),
                                          accel_bound.second);

    bool has_follow = false;
    double delta_s = 1.0;
    for (const auto &boundary : boundaries)
    {
        if (boundary.boundary_type() == ST_Boundary::BoundaryType::FOLLOW)
        {
            has_follow = true;
            delta_s = std::fmin(
                delta_s, boundary.min_s_ - fabs(boundary.characteristic_length()));
        }
    }
    if (Config_.FLAGS_enable_follow_accel_constraint && has_follow && delta_s < 0.0)
    {
        accel_upper_bound.front() = 0.0;
    }

    // DCHECK_EQ(t_evaluated_.size(), accel_lower_bound.size());
    // DCHECK_EQ(t_evaluated_.size(), accel_upper_bound.size());
    if (!constraint->AddSecondDerivativeBoundary(t_evaluated_, accel_lower_bound,
                                                 accel_upper_bound))
    {
        const std::string msg = "Fail to apply acceleration constraints.";
        return false;
    }
    // for (size_t i = 0; i < t_evaluated_.size(); ++i)
    // {
    //     std::cout << "t_evaluated_: " << t_evaluated_[i]
    //     << "; accel_lower_bound: " << accel_lower_bound[i]
    //     << "; accel_upper_bound: " << accel_upper_bound[i]<<"\n";
    // }

    return true;
}

bool QpSplineStGraph::AddKernel(
    const std::vector<ST_Boundary> &boundaries,
    const planning::SpeedLimit &speed_limit)
{
    Spline1dKernel *spline_kernel = spline_solver_->mutable_spline_kernel();

    if (Config_.accel_kernel_weight > 0)
    {
        //1、优化二阶导，即减小加速度
        spline_kernel->AddSecondOrderDerivativeMatrix(
            Config_.accel_kernel_weight);
    }

    if (Config_.jerk_kernel_weight > 0)
    {
        //2、优化三阶导，即减小jerk
        spline_kernel->AddThirdOrderDerivativeMatrix(
            Config_.jerk_kernel_weight);
    }
    //3、优化目标：cruise对应的参考速度，即以最短时间、最快速度行驶
    if (!AddCruiseReferenceLineKernel(Config_.cruise_weight))
    {
        return false;
    }
    //4、优化目标：follow决策的参考速度
    if (!AddFollowReferenceLineKernel(boundaries, Config_.follow_weight))
    {
        return false;
    }
    //5、优化目标：yield决策的参考速度
    if (!AddYieldReferenceLineKernel(boundaries, Config_.yield_weight))
    {
        return false;
    }
    //6、优化目标：EM的DP输出的参考速度
    if (!AddDpStReferenceKernel(Config_.dp_st_reference_weight))
    {
        return false;
    }
    // init point jerk continuous kernel 看不懂？？
    (*spline_kernel->mutable_kernel_matrix())(2, 2) +=
        2.0 * 4.0 * Config_.init_jerk_kernel_weight;
    (*spline_kernel->mutable_offset())(2, 0) +=
        -4.0 * init_point_.a * Config_.init_jerk_kernel_weight;

    //7、正则项
    spline_kernel->AddRegularization(Config_.regularization_weight);

    return true;
}

bool QpSplineStGraph::Solve()
{
    return spline_solver_->Solve()
               ? true
               : false;
}

bool QpSplineStGraph::AddCruiseReferenceLineKernel(const double weight)
{
    auto *spline_kernel = spline_solver_->mutable_spline_kernel();
    double dist_ref = Config_.total_path_length;
    for (uint32_t i = 0; i < t_evaluated_.size(); ++i)
    {
        cruise_.push_back(dist_ref);
    }

    // DCHECK_EQ(t_evaluated_.size(), cruise_.size());

    // for (size_t i = 0; i < t_evaluated_.size(); ++i)
    // {
    //     std::cout << "Cruise Ref S: " << cruise_[i]
    //               << " Relative time: " << t_evaluated_[i] << std::endl;
    // }

    if (t_evaluated_.size() > 0)
    {
        spline_kernel->AddReferenceLineKernelMatrix(
            t_evaluated_, cruise_, weight * Config_.total_time / static_cast<double>(t_evaluated_.size()));
    }

    return true;
}

bool QpSplineStGraph::AddFollowReferenceLineKernel(
    const std::vector<ST_Boundary> &boundaries, const double weight)
{
    auto *spline_kernel = spline_solver_->mutable_spline_kernel();
    std::vector<double> ref_s;
    std::vector<double> filtered_evaluate_t;
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
        }
    }
    // DCHECK_EQ(filtered_evaluate_t.size(), ref_s.size());

    if (!ref_s.empty())
    {
        spline_kernel->AddReferenceLineKernelMatrix(
            filtered_evaluate_t, ref_s,
            weight * Config_.total_time /
                static_cast<double>(t_evaluated_.size()));
    }

    for (size_t i = 0; i < filtered_evaluate_t.size(); ++i)
    {
        //std::cout << "Follow Ref S: " << ref_s[i]
        // << " Relative time: " << filtered_evaluate_t[i] << std::endl;
    }
    return true;
}

bool QpSplineStGraph::AddYieldReferenceLineKernel(
    const std::vector<ST_Boundary> &boundaries, const double weight)
{
    auto *spline_kernel = spline_solver_->mutable_spline_kernel();
    std::vector<double> ref_s;
    std::vector<double> filtered_evaluate_t;
    for (size_t i = 0; i < t_evaluated_.size(); ++i)
    {
        const double curr_t = t_evaluated_[i];
        double s_min = std::numeric_limits<double>::infinity();
        bool success = false;
        for (const auto &boundary : boundaries)
        {
            if (boundary.boundary_type() != ST_Boundary::BoundaryType::YIELD)
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
                s_min = std::min(s_min, s_upper - boundary.characteristic_length() - Config_.yield_drag_distance);
            }
        }
        if (success && s_min < cruise_[i])
        {
            filtered_evaluate_t.push_back(curr_t);
            ref_s.push_back(s_min);
        }
    }
    // DCHECK_EQ(filtered_evaluate_t.size(), ref_s.size());

    if (!ref_s.empty())
    {
        spline_kernel->AddReferenceLineKernelMatrix(
            filtered_evaluate_t, ref_s, weight * Config_.total_time / static_cast<double>(t_evaluated_.size()));
    }

    for (size_t i = 0; i < filtered_evaluate_t.size(); ++i)
    {
        //std::cout << "Yield Ref S: " << ref_s[i]
        // << " Relative time: " << filtered_evaluate_t[i] << std::endl;
    }
    return true;
}

bool QpSplineStGraph::AddDpStReferenceKernel(const double weight) const
{
    std::vector<double> t_pos;
    std::vector<double> s_pos;
    for (auto point : reference_dp_speed_points_)
    {
        t_pos.push_back(point.t);
        s_pos.push_back(point.s);
    }
    auto *spline_kernel = spline_solver_->mutable_spline_kernel();
    if (!t_pos.empty())
    {
        spline_kernel->AddReferenceLineKernelMatrix(
            t_pos, s_pos, weight * Config_.total_time / static_cast<double>(t_pos.size()));
    }
    return true;
}

bool QpSplineStGraph::GetSConstraintByTime(
    const std::vector<ST_Boundary> &boundaries, const double time,
    const double total_path_s, double *const s_upper_bound,
    double *const s_lower_bound) const
{
    *s_upper_bound = total_path_s;

    for (const auto &boundary : boundaries)
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
        else if (boundary.boundary_type() ==
                 ST_Boundary::BoundaryType::OVERTAKE)
        {
            *s_lower_bound = std::fmax(*s_lower_bound, s_lower);
        }
        else
        {
            std::cout << "Boundary_Type:" << boundary.TypeName() << "\n";
        }
    }

    return true;
}

const SpeedData QpSplineStGraph::GetHistorySpeed() const
{
    const auto speed_data = reference_line_info_.speed_data();
    if (speed_data.empty())
    {
        //  std::cout << "last frame is empty";
        return SpeedData();
    }

    return speed_data;
}

/*
这个过程也比较简单，只要计算t_evaluated_中每个点对应的上下界，下届为0，
上界为距离该点最近一个限速区的限速。最后同5一样，每个点的速度，也就是函数一阶导约束在上下界之间。
*/
bool QpSplineStGraph::EstimateSpeedUpperBound(
    const TrajectoryPoint &init_point, const planning::SpeedLimit &speed_limit,
    std::vector<double> *speed_upper_bound) const
{
    // DCHECK_NOTNULL(speed_upper_bound);

    speed_upper_bound->clear();

    // use v to estimate position: not accurate, but feasible in cyclic
    // processing. We can do the following process multiple times and use
    // previous cycle's results for better estimation.
    const double v = init_point.v;
    auto last_speed_data = GetHistorySpeed();

    if (static_cast<double>(t_evaluated_.size() +
                            speed_limit.speed_limit_points().size()) <
        static_cast<double>(t_evaluated_.size()) *
            std::log(
                static_cast<double>(speed_limit.speed_limit_points().size())))
    {
        uint32_t i = 0;
        uint32_t j = 0;
        while (i < t_evaluated_.size() &&
               j + 1 < speed_limit.speed_limit_points().size())
        {
            double distance = v * t_evaluated_[i];
            //与上一时刻的轨迹速度进行比较
            if (!last_speed_data.empty() && distance < last_speed_data.back().s)
            {
                SpeedPoint p;
                last_speed_data.EvaluateByTime(t_evaluated_[i], &p);
                distance = p.s;
            }
            constexpr double kDistanceEpsilon = 1e-6;
            if (fabs(distance - speed_limit.speed_limit_points()[j].first) <
                kDistanceEpsilon)
            {
                speed_upper_bound->push_back(
                    speed_limit.speed_limit_points()[j].second);
                ++i;
            }
            else if (distance < speed_limit.speed_limit_points()[j].first)
            {
                ++i;
            }
            else if (distance <= speed_limit.speed_limit_points()[j + 1].first)
            {
                speed_upper_bound->push_back(speed_limit.GetSpeedLimitByS(distance));
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
            //std::cout << "speed upper bound:" << speed_upper_bound->back();
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
            double s = v * t;

            //与上一时刻的轨迹速度进行比较
            if (!last_speed_data.empty() && s < last_speed_data.back().s)
            {
                SpeedPoint p;
                last_speed_data.EvaluateByTime(t, &p);
                s = p.s;
            }

            // NOTICE: we are using binary search here based on two assumptions:
            // (1) The s in speed_limit_points increase monotonically.
            // (2) The evaluated_t_.size() << number of speed_limit_points.size()
            //
            // If either of the two assumption is failed, a new algorithm must be
            // used to replace the binary search.

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

    if (is_change_lane_)
    {
        for (uint32_t k = 0; k < t_evaluated_.size(); ++k)
        {
            speed_upper_bound->at(k) *=
                (1.0 + Config_.change_lane_speed_relax_percentage);
        }
    }

    const double kTimeBuffer = 1.0;
    const double kSpeedBuffer = 0.1;
    for (uint32_t k = 0; k < t_evaluated_.size() && t_evaluated_[k] < kTimeBuffer;
         ++k)
    {
        speed_upper_bound->at(k) =
            std::fmax(init_point_.v + kSpeedBuffer, speed_upper_bound->at(k));
    }

    return true;
}