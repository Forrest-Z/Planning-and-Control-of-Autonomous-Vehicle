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
 * Modified function input and used only some functions
 **/

#include "hybrid_a_star.h"
#include "piecewise_jerk_speed_problem.h"
#include "point_factory.h"

HybridAStar::HybridAStar()
{
    reed_shepp_generator_ = std::make_unique<ReedShepp>();
    grid_a_star_heuristic_generator_ = std::make_unique<GridSearch>();
    next_node_num_ = Config_.next_node_num;
    max_steer_angle_ = Config_.max_steer_angle / Config_.steer_ratio;
    step_size_ = Config_.step_size;
    xy_grid_resolution_ = Config_.xy_grid_resolution;
    delta_t_ = Config_.delta_t;
    traj_forward_penalty_ = Config_.traj_forward_penalty;
    traj_back_penalty_ = Config_.traj_back_penalty;
    traj_gear_switch_penalty_ = Config_.traj_gear_switch_penalty;
    traj_steer_penalty_ = Config_.traj_steer_penalty;
    traj_steer_change_penalty_ = Config_.traj_steer_change_penalty;
}

// 尝试使用ReedShepp曲线连接当前点与目标点，若成功，则Hybrid A*规划完成
// 允许返回false，其实只返回一次true
bool HybridAStar::AnalyticExpansion(std::shared_ptr<Node3d> current_node)
{
    std::shared_ptr<ReedSheppPath> reeds_shepp_to_check = std::make_shared<ReedSheppPath>();
    // ReedShepp曲线都是从当前点到终点的
    if (!reed_shepp_generator_->ShortestRSP(current_node, end_node_,
                                            reeds_shepp_to_check))
    {
        std::cout << "ShortestRSP failed";
        return false;
    }

    // ReedShepp曲线段的碰撞检测与越界检测
    if (!RSPCheck(reeds_shepp_to_check))
    {
        return false;
    }

    // std::cout << "Reach the end configuration with Reed Sharp";
    //  load the whole RSP as nodes and add to the close set
    // 将连接到目标点的一段ReedShepp曲线封装成node，放入Hybrid A*的集合中
    final_node_ = LoadRSPinCS(reeds_shepp_to_check, current_node);

    return true;
}

bool HybridAStar::RSPCheck(
    const std::shared_ptr<ReedSheppPath> reeds_shepp_to_end)
{
    std::shared_ptr<Node3d> node = std::shared_ptr<Node3d>(new Node3d(
        reeds_shepp_to_end->x, reeds_shepp_to_end->y, reeds_shepp_to_end->phi,
        XYbounds_));
    return ValidityCheck(node);
}

bool HybridAStar::ValidityCheck(std::shared_ptr<Node3d> node)
{
    // CHECK_NOTNULL(node);
    // CHECK_GT(node->GetStepSize(), 0);

    if (obstacles_linesegments_vec_.empty())
    {
        return true;
    }

    size_t node_step_size = node->GetStepSize();
    const auto &traversed_x = node->GetXs();
    const auto &traversed_y = node->GetYs();
    const auto &traversed_phi = node->GetPhis();

    // The first {x, y, phi} is collision free unless they are start and end
    // configuration of search problem
    size_t check_start_index = 0;
    if (node_step_size == 1)
    {
        check_start_index = 0;
    }
    else
    {
        check_start_index = 1;
    }

    for (size_t i = check_start_index; i < node_step_size; ++i)
    {
        if (traversed_x[i] > XYbounds_[1] || traversed_x[i] < XYbounds_[0] ||
            traversed_y[i] > XYbounds_[3] || traversed_y[i] < XYbounds_[2])
        {
            return false;
        }
        Box2d bounding_box = Node3d::GetBoundingBox(traversed_x[i], traversed_y[i], traversed_phi[i]);
        for (const auto &obstacle_linesegments : obstacles_linesegments_vec_)
        {
            for (const LineSegment2d &linesegment :
                 obstacle_linesegments)
            {
                if (bounding_box.HasOverlap(linesegment))
                {
                    // std::cout << "collision start at x: " << linesegment.start().x();
                    // std::cout << "collision start at y: " << linesegment.start().y();
                    // std::cout << "collision end at x: " << linesegment.end().x();
                    // std::cout << "collision end at y: " << linesegment.end().y();
                    return false;
                }
            }
        }
    }
    return true;
}

std::shared_ptr<Node3d> HybridAStar::LoadRSPinCS(
    const std::shared_ptr<ReedSheppPath> reeds_shepp_to_end,
    std::shared_ptr<Node3d> current_node)
{
    std::shared_ptr<Node3d> end_node = std::shared_ptr<Node3d>(new Node3d(
        reeds_shepp_to_end->x, reeds_shepp_to_end->y, reeds_shepp_to_end->phi,
        XYbounds_));
    end_node->SetPre(current_node);
    close_set_.emplace(end_node->GetIndex(), end_node);
    return end_node;
}

// 扩展节点的重点在于把车辆运动学模型的约束考虑进去，根据限定的steering angle 去搜索相邻的grid。
// 扩展节点，扩展一个node就是扩展了一个grid，但是会产生多个在同一grid内的路径点
std::shared_ptr<Node3d> HybridAStar::Next_node_generator(
    std::shared_ptr<Node3d> current_node, size_t next_node_index)
{
    double steering = 0.0;
    size_t index = 0;
    double traveled_distance = 0.0;

    // steering angle为什么这么算？
    // 首先，根据next_node_index与next_node_num_的对比是可以区分运动方向的
    // 这里的if-else就是区分运动正反方向讨论的（前进和倒车）
    // 其次，车辆在当前的姿态下，既可以向左转、又可以向右转，那么steering angle的
    // 取值范围其实是[-max_steer_angle_, max_steer_angle_]，在正向或反向下，
    // 能取next_node_num_/2个有效值。
    // 即，把[-max_steer_angle_, max_steer_angle_]分为（next_node_num_/2-1）份
    // 所以，steering = 初始偏移量 + 单位间隔 × index
    // steering angle的正负取决于车的转向，而非前进的正反
    if (next_node_index < static_cast<double>(next_node_num_) / 2)
    {
        steering =
            -max_steer_angle_ +
            (2 * max_steer_angle_ / (static_cast<double>(next_node_num_) / 2 - 1)) *
                static_cast<double>(next_node_index);
        traveled_distance = step_size_;
    }
    else
    {
        index = next_node_index - next_node_num_ / 2;
        steering =
            -max_steer_angle_ +
            (2 * max_steer_angle_ / (static_cast<double>(next_node_num_) / 2 - 1)) *
                static_cast<double>(index);
        traveled_distance = -step_size_;
    }
    // take above motion primitive to generate a curve driving the car to a
    // different grid
    double arc = std::sqrt(2) * xy_grid_resolution_;
    std::vector<double> intermediate_x;
    std::vector<double> intermediate_y;
    std::vector<double> intermediate_phi;
    double last_x = current_node->GetX();
    double last_y = current_node->GetY();
    double last_phi = current_node->GetPhi();
    intermediate_x.push_back(last_x);
    intermediate_y.push_back(last_y);
    intermediate_phi.push_back(last_phi);
    // 从当前grid前进到下一个grid，一个grid内可能有多个路径点
    for (size_t i = 0; i < arc / step_size_; ++i)
    {
        const double next_x = last_x + traveled_distance * std::cos(last_phi);
        const double next_y = last_y + traveled_distance * std::sin(last_phi);
        const double next_phi = common::math::NormalizeAngle(
            last_phi +
            traveled_distance / Config_.wheel_base * std::tan(steering));
        intermediate_x.push_back(next_x);
        intermediate_y.push_back(next_y);
        intermediate_phi.push_back(next_phi);
        last_x = next_x;
        last_y = next_y;
        last_phi = next_phi;
    }
    // check if the vehicle runs outside of XY boundary
    if (intermediate_x.back() > XYbounds_[1] ||
        intermediate_x.back() < XYbounds_[0] ||
        intermediate_y.back() > XYbounds_[3] ||
        intermediate_y.back() < XYbounds_[2])
    {
        return nullptr;
    }
    std::shared_ptr<Node3d> next_node = std::shared_ptr<Node3d>(
        new Node3d(intermediate_x, intermediate_y, intermediate_phi, XYbounds_));
    next_node->SetPre(current_node);
    next_node->SetDirec(traveled_distance > 0.0);
    next_node->SetSteer(steering);
    return next_node;
}

// 分别计算路径代价和启发代价，就是A*中的G和H。这里计算启发代价的方法很巧妙，用了DP
void HybridAStar::CalculateNodeCost(std::shared_ptr<Node3d> current_node,
                                    std::shared_ptr<Node3d> next_node)
{
    // A*中走过的轨迹的代价G, f = h + g 中的 "g"
    next_node->SetTrajCost(current_node->GetTrajCost() +
                           TrajCost(current_node, next_node));
    // evaluate heuristic cost
    double optimal_path_cost = 0.0;
    // A*中从当前点到目标点的启发式代价H，采用了动态规划DP来计算（以目标点为DP的起点）,f = h + g 中的 "h"
    optimal_path_cost += HoloObstacleHeuristic(next_node);
    next_node->SetHeuCost(optimal_path_cost);
}

double HybridAStar::TrajCost(std::shared_ptr<Node3d> current_node,
                             std::shared_ptr<Node3d> next_node)
{
    // evaluate cost on the trajectory and add current cost
    double piecewise_cost = 0.0;
    if (next_node->GetDirec())
    {
        piecewise_cost += static_cast<double>(next_node->GetStepSize() - 1) *
                          step_size_ * traj_forward_penalty_;
    }
    else
    {
        piecewise_cost += static_cast<double>(next_node->GetStepSize() - 1) *
                          step_size_ * traj_back_penalty_;
    }
    if (current_node->GetDirec() != next_node->GetDirec())
    {
        piecewise_cost += traj_gear_switch_penalty_;
    }
    piecewise_cost += traj_steer_penalty_ * std::abs(next_node->GetSteer());
    piecewise_cost += traj_steer_change_penalty_ *
                      std::abs(next_node->GetSteer() - current_node->GetSteer());
    return piecewise_cost;
}

// 计算 f = h + g 中的 "h"，调用DP_Map得到，其实就是以终点为start的Dijkstra方法
double HybridAStar::HoloObstacleHeuristic(std::shared_ptr<Node3d> next_node)
{
    return grid_a_star_heuristic_generator_->CheckDpMap(next_node->GetX(),
                                                        next_node->GetY());
}

// 回溯法得到路径，从final_node_顺藤摸瓜
bool HybridAStar::GetResult(HybridAStartResult *result)
{
    std::shared_ptr<Node3d> current_node = final_node_;
    std::vector<double> hybrid_a_x;
    std::vector<double> hybrid_a_y;
    std::vector<double> hybrid_a_phi;
    while (current_node->GetPreNode() != nullptr)
    {
        std::vector<double> x = current_node->GetXs();
        std::vector<double> y = current_node->GetYs();
        std::vector<double> phi = current_node->GetPhis();
        if (x.empty() || y.empty() || phi.empty())
        {
            // std::cout << "result size check failed";
            return false;
        }
        if (x.size() != y.size() || x.size() != phi.size())
        {
            // std::cout << "states sizes are not equal";
            return false;
        }
        std::reverse(x.begin(), x.end());
        std::reverse(y.begin(), y.end());
        std::reverse(phi.begin(), phi.end());
        x.pop_back();
        y.pop_back();
        phi.pop_back();
        hybrid_a_x.insert(hybrid_a_x.end(), x.begin(), x.end());
        hybrid_a_y.insert(hybrid_a_y.end(), y.begin(), y.end());
        hybrid_a_phi.insert(hybrid_a_phi.end(), phi.begin(), phi.end());
        current_node = current_node->GetPreNode();
    }
    hybrid_a_x.push_back(current_node->GetX());
    hybrid_a_y.push_back(current_node->GetY());
    hybrid_a_phi.push_back(current_node->GetPhi());

    // 上面traversed_x_ y_ phi_被反转了，得反转回来
    std::reverse(hybrid_a_x.begin(), hybrid_a_x.end());
    std::reverse(hybrid_a_y.begin(), hybrid_a_y.end());
    std::reverse(hybrid_a_phi.begin(), hybrid_a_phi.end());
    // 此时 result只包含位姿信息 x,y,phi
    (*result).x = hybrid_a_x;
    (*result).y = hybrid_a_y;
    (*result).phi = hybrid_a_phi;

    // 添加速度、加速度、转向信息 v,a,steering
    if (!GetTemporalProfile(result))
    {
        std::cout << "GetSpeedProfile from Hybrid Astar path fails";
        return false;
    }

    if (result->x.size() != result->y.size() ||
        result->x.size() != result->v.size() ||
        result->x.size() != result->phi.size())
    {
        // std::cout << "state sizes not equal, "
        //     << "result->x.size(): " << result->x.size() << "result->y.size()"
        //     << result->y.size() << "result->phi.size()" << result->phi.size()
        //     << "result->v.size()" << result->v.size();
        return false;
    }
    if (result->a.size() != result->steer.size() ||
        result->x.size() - result->a.size() != 1)
    {
        // std::cout << "control sizes not equal or not right";
        // std::cout << " acceleration size: " << result->a.size();
        // std::cout << " steer size: " << result->steer.size();
        // std::cout << " x size: " << result->x.size();
        return false;
    }
    return true;
}

bool HybridAStar::GenerateSpeedAcceleration(HybridAStartResult *result)
{
    // Sanity Check
    if (result->x.size() < 2 || result->y.size() < 2 || result->phi.size() < 2)
    {
        // std::cout << "result size check when generating speed and acceleration fail";
        return false;
    }
    const size_t x_size = result->x.size();

    // load velocity from position
    // initial and end speed are set to be zeros
    // 从位置恢复速度 v
    // 起点、终点的 v 为 “零”，因为不同段之间换挡时，肯定要把车刹住才能换挡
    result->v.push_back(0.0);
    for (size_t i = 1; i + 1 < x_size; ++i)
    {
        // 求s轴速度，不同方向的速度怎么可以直接相加求合速度呢？
        double discrete_v = (((result->x[i + 1] - result->x[i]) / delta_t_) *
                                 std::cos(result->phi[i]) +
                             ((result->x[i] - result->x[i - 1]) / delta_t_) *
                                 std::cos(result->phi[i])) /
                                2.0 +
                            // 上面是x方向上，利用连续3点的坐标求中间点的速度，平均速度
                            (((result->y[i + 1] - result->y[i]) / delta_t_) *
                                 std::sin(result->phi[i]) +
                             ((result->y[i] - result->y[i - 1]) / delta_t_) *
                                 std::sin(result->phi[i])) /
                                2.0;
        // 上面是y方向上，利用连续3点的坐标求中间点的速度，平均速度
        result->v.push_back(discrete_v);
    }
    result->v.push_back(0.0);

    // load acceleration from velocity
    for (size_t i = 0; i + 1 < x_size; ++i)
    {
        const double discrete_a = (result->v[i + 1] - result->v[i]) / delta_t_;
        result->a.push_back(discrete_a);
    }

    // load steering from phi
    for (size_t i = 0; i + 1 < x_size; ++i)
    {
        double discrete_steer = (result->phi[i + 1] - result->phi[i]) *
                                Config_.wheel_base / step_size_;
        if (result->v[i] > 0.0)
        {
            discrete_steer = std::atan(discrete_steer);
        }
        else
        {
            discrete_steer = std::atan(-discrete_steer);
        }
        result->steer.push_back(discrete_steer);
    }
    return true;
}

// 分割好的每一小段路径中，累加得到s后，使用PiecewiseJerkSpeedProblem得到v、a（其实就是用jerk恒定的三次多项式曲线来拟合）
/*
目标函数：s'惩罚，s''惩罚，s'''惩罚、末态惩罚、与ref误差的惩罚

决策变量：s、ds（速度）、dds（加速度）

变量个数：total_t / delta_t * 3
*/
bool HybridAStar::GenerateSCurveSpeedAcceleration(HybridAStartResult *result)
{
    // sanity check
    // CHECK_NOTNULL(result);
    if (result->x.size() < 2 || result->y.size() < 2 || result->phi.size() < 2)
    {
        std::cout << "result size check when generating speed and acceleration fail";
        return false;
    }
    if (result->x.size() != result->y.size() ||
        result->x.size() != result->phi.size())
    {
        std::cout << "result sizes not equal";
        return false;
    }

    // get gear info
    double init_heading = result->phi.front();
    const Vec2d init_tracking_vector(result->x[1] - result->x[0],
                                     result->y[1] - result->y[0]);
    const double gear =
        std::abs(common::math::NormalizeAngle(
            init_heading - init_tracking_vector.Angle())) < M_PI_2;

    // get path lengh
    size_t path_points_size = result->x.size();

    double accumulated_s = 0.0;
    result->accumulated_s.clear();
    auto last_x = result->x.front();
    auto last_y = result->y.front();

    // 累加得到这段路径的长度 accumulcccated_s
    for (size_t i = 0; i < path_points_size; ++i)
    {
        double x_diff = result->x[i] - last_x;
        double y_diff = result->y[i] - last_y;
        accumulated_s += std::sqrt(x_diff * x_diff + y_diff * y_diff);
        result->accumulated_s.push_back(accumulated_s);
        last_x = result->x[i];
        last_y = result->y[i];
    }

    // assume static initial state
    const double init_v = 0.0; // 这里就是导致起点速度为0的原因，也可以改成default_init_speed
    const double init_a = 0.0;

    // minimum time speed optimization
    // TODO(Jinyun): move to confs
    const double max_forward_v = Config_.max_forward_v;
    const double max_reverse_v = Config_.max_reverse_v;
    const double max_forward_acc = Config_.max_forward_acc;
    const double max_reverse_acc = Config_.max_reverse_acc;
    const double max_acc_jerk = Config_.max_acc_jerk;
    const double delta_t = Config_.delta_t;

    SpeedData speed_data;
    // TODO(Jinyun): explore better time horizon heuristic
    const double path_length = result->accumulated_s.back();
    // std::cout << "path_length:" << path_length << "\n";
    const double total_t = std::max(gear ? 1.5 *
                                               (max_forward_v * max_forward_v +
                                                path_length * max_forward_acc) /
                                               (max_forward_acc * max_forward_v)
                                         : 1.5 *
                                               (max_reverse_v * max_reverse_v +
                                                path_length * max_reverse_acc) /
                                               (max_reverse_acc * max_reverse_v),
                                    10.0);

    const size_t num_of_knots = static_cast<size_t>(total_t / delta_t) + 1;
    // std::cout << "total_t:" << total_t << "\n";

    // std::cout << "num_of_knots:" << num_of_knots << "\n";

    PiecewiseJerkSpeedProblem piecewise_jerk_problem(
        num_of_knots, delta_t, {0.0, std::abs(init_v), std::abs(init_a)});

    // set constraints
    std::vector<std::pair<double, double>> x_bounds(num_of_knots, {0.0, path_length});

    const double max_v = gear ? max_forward_v : max_reverse_v;
    const double max_acc = gear ? max_forward_acc : max_reverse_acc;

    const auto upper_dx = std::fmax(max_v, std::abs(init_v));
    std::vector<std::pair<double, double>> dx_bounds(num_of_knots,
                                                     {0.0, upper_dx});
    std::vector<std::pair<double, double>> ddx_bounds(num_of_knots,
                                                      {-max_acc, max_acc});
    // set end constraints
    x_bounds[num_of_knots - 1] = std::make_pair(path_length, path_length);
    dx_bounds[num_of_knots - 1] = std::make_pair(0.0, 0.0);
    ddx_bounds[num_of_knots - 1] = std::make_pair(0.0, 0.0);

    // TODO(Jinyun): move to confs
    std::vector<double> x_ref(num_of_knots, path_length);
    piecewise_jerk_problem.set_x_ref(10000.0, std::move(x_ref));
    piecewise_jerk_problem.set_weight_ddx(10.0);
    piecewise_jerk_problem.set_weight_dddx(10.0);
    piecewise_jerk_problem.set_x_bounds(std::move(x_bounds));
    piecewise_jerk_problem.set_dx_bounds(std::move(dx_bounds));
    piecewise_jerk_problem.set_ddx_bounds(std::move(ddx_bounds));
    piecewise_jerk_problem.set_dddx_bound(max_acc_jerk);

    // solve the problem
    if (!piecewise_jerk_problem.Optimize(4000))
    {
        std::cout << "Piecewise jerk speed optimizer failed!";
        return false;
    }

    // extract output
    const std::vector<double> &s = piecewise_jerk_problem.opt_x();
    const std::vector<double> &ds = piecewise_jerk_problem.opt_dx();
    const std::vector<double> &dds = piecewise_jerk_problem.opt_ddx();

    // assign speed point by gear
    speed_data.AppendSpeedPoint(s[0], 0.0, ds[0], dds[0], 0.0);
    const double kEpislon = 1.0e-6;
    const double sEpislon = 1.0e-6;

    for (size_t i = 1; i < num_of_knots; ++i)
    {
        if (s[i - 1] - s[i] > kEpislon)
        {
            // std::cout << "unexpected decreasing s in speed smoothing at time "
            //  << static_cast<double>(i) * delta_t << "with total time "
            //  << total_t;
            break;
        }
        speed_data.AppendSpeedPoint(s[i], delta_t * static_cast<double>(i), ds[i],
                                    dds[i], (dds[i] - dds[i - 1]) / delta_t);
        // cut the speed data when it is about to meet end condition
        if (path_length - s[i] < sEpislon)
        {
            break;
        }
    }

    // combine speed and path profile
    DiscretizedPath path_data;
    for (size_t i = 0; i < path_points_size; ++i)
    {
        PathPoint path_point;
        path_point.set_x(result->x[i]);
        path_point.set_y(result->y[i]);
        path_point.set_theta(result->phi[i]);
        path_point.set_s(result->accumulated_s[i]);
        path_data.push_back(std::move(path_point));
    }

    HybridAStartResult combined_result;

    // TODO(Jinyun): move to confs
    const double kDenseTimeResoltuion = 0.5;
    const double time_horizon =
        speed_data.TotalTime() + kDenseTimeResoltuion * 1.0e-6;
    if (path_data.empty())
    {
        std::cout << "path data is empty";
        return false;
    }
    for (double cur_rel_time = 0.0; cur_rel_time < time_horizon;
         cur_rel_time += kDenseTimeResoltuion)
    {
        SpeedPoint speed_point;
        if (!speed_data.EvaluateByTime(cur_rel_time, &speed_point))
        {
            std::cout << "Fail to get speed point with relative time " << cur_rel_time;
            return false;
        }

        if (speed_point.s > path_data.Length())
        {
            break;
        }

        PathPoint path_point = path_data.Evaluate(speed_point.s);

        combined_result.x.push_back(path_point.x);
        combined_result.y.push_back(path_point.y);
        combined_result.phi.push_back(path_point.theta);
        combined_result.accumulated_s.push_back(path_point.s);
        if (!gear)
        {
            combined_result.v.push_back(-speed_point.v);
            combined_result.a.push_back(-speed_point.a);
        }
        else
        {
            combined_result.v.push_back(speed_point.v);
            combined_result.a.push_back(speed_point.a);
        }
    }

    combined_result.a.pop_back();

    // recalc step size
    path_points_size = combined_result.x.size();

    // load steering from phi
    for (size_t i = 0; i + 1 < path_points_size; ++i)
    {
        double discrete_steer =
            (combined_result.phi[i + 1] - combined_result.phi[i]) * Config_.wheel_base /
            (combined_result.accumulated_s[i + 1] -
             combined_result.accumulated_s[i]);
        discrete_steer =
            gear ? std::atan(discrete_steer) : std::atan(-discrete_steer);
        combined_result.steer.push_back(discrete_steer);
    }

    *result = combined_result;
    return true;
}

// 将Hybrid A*计算的轨迹结果，按照行驶的正反方向切换，分割为数段，
// 并完善轨迹的静态、动态信息
bool HybridAStar::TrajectoryPartition(
    const HybridAStartResult &result,
    std::vector<HybridAStartResult> *partitioned_result)
{
    const auto &x = result.x;
    const auto &y = result.y;
    const auto &phi = result.phi;
    if (x.size() != y.size() || x.size() != phi.size())
    {
        std::cout << "states sizes are not equal when do trajectory partitioning of "
                     "Hybrid A Star result";
        return false;
    }

    size_t horizon = x.size();
    partitioned_result->clear();
    partitioned_result->emplace_back();
    auto *current_traj = &(partitioned_result->back());
    double heading_angle = phi.front();
    const Vec2d init_tracking_vector(x[1] - x[0], y[1] - y[0]);
    double tracking_angle = init_tracking_vector.Angle();
    bool current_gear =
        std::abs(common::math::NormalizeAngle(tracking_angle - heading_angle)) <
        (M_PI_2);
    // 此时的result只有路径静态信息，x,y,phi
    // 将Hybrid A*计算的轨迹结果，按照行驶的正反方向切换，分割为数段
    for (size_t i = 0; i < horizon - 1; ++i)
    {
        heading_angle = phi[i];
        const Vec2d tracking_vector(x[i + 1] - x[i], y[i + 1] - y[i]);
        tracking_angle = tracking_vector.Angle();
        // 因为规划的路径中，有的是”前进“有的是”倒退“，可以使用路径朝向和车身朝向的夹角来判断“前进”还是“倒退”完成轨迹的分段
        //  路径朝向和车身朝向的夹角来判断“前进”还是“倒退”
        bool gear =
            std::abs(common::math::NormalizeAngle(tracking_angle - heading_angle)) <
            (M_PI_2);
        // 换 gear，这一段轨迹就完成了，重启一段
        if (gear != current_gear)
        {
            current_traj->x.push_back(x[i]);
            current_traj->y.push_back(y[i]);
            current_traj->phi.push_back(phi[i]);
            partitioned_result->emplace_back();
            current_traj = &(partitioned_result->back());
            current_gear = gear; // 更新 current_gear
        }
        // 不换 gear
        current_traj->x.push_back(x[i]);
        current_traj->y.push_back(y[i]);
        current_traj->phi.push_back(phi[i]);
    }
    current_traj->x.push_back(x.back());
    current_traj->y.push_back(y.back());
    current_traj->phi.push_back(phi.back());

    const auto start_timestamp = std::chrono::system_clock::now();

    // std::cout << "partitioned_result:" << partitioned_result->size() << "\n";

    // Retrieve v, a and steer from path
    for (auto &result : *partitioned_result)
    {
        // 2种不同的方式获取轨迹动态信息，v,a,steer。区别: 前者用数值优化的方法，后者用相邻点静态信息
        if (Config_.FLAGS_use_s_curve_speed_smooth)
        {
            // 使用QP优化方法求frenet系下的轨迹，但是结果只有动态信息 s,v,a,steer
            if (!GenerateSCurveSpeedAcceleration(&result))
            {
                std::cout << "GenerateSCurveSpeedAcceleration fail";
                return false;
            }
        }
        else
        {
            if (!GenerateSpeedAcceleration(&result))
            {
                // std::cout << "GenerateSpeedAcceleration fail";
                return false;
            }
        }
    }

    const auto end_timestamp = std::chrono::system_clock::now();
    std::chrono::duration<double> diff = end_timestamp - start_timestamp;
    // std::cout << "speed profile total time: " << diff.count() * 1000.0 << " ms.";
    return true;
}

/*
初步路径的每个node都指向上一个node，HybridAStar::GetResult() 在把这些node反向后（由当前node指向终点node），得到了顺序正确的node集合。
注意，此时形成了一个挨一个的node，还不是一个挨一个的轨迹点。因此，要调用 GetTemporalProfile(result) 来完成大部分的后处理，得到最终结果。
*/
// 将Hybrid A*计算的轨迹结果，按照行驶的正反方向切换，分割为数段，分别逆向翻转轨迹点
// 然后重新拼接在一起，就是最终可以发布供车行驶的轨迹
bool HybridAStar::GetTemporalProfile(HybridAStartResult *result)
{
    std::vector<HybridAStartResult> partitioned_results;
    if (!TrajectoryPartition(*result, &partitioned_results))
    {
        std::cout << "TrajectoryPartition fail";
        return false;
    }
    // 将分段的轨迹拼接起来
    HybridAStartResult stitched_result;
    for (const auto &result : partitioned_results)
    {
        std::copy(result.x.begin(), result.x.end() - 1,
                  std::back_inserter(stitched_result.x));
        std::copy(result.y.begin(), result.y.end() - 1,
                  std::back_inserter(stitched_result.y));
        std::copy(result.phi.begin(), result.phi.end() - 1,
                  std::back_inserter(stitched_result.phi));
        std::copy(result.v.begin(), result.v.end() - 1,
                  std::back_inserter(stitched_result.v));
        std::copy(result.a.begin(), result.a.end(),
                  std::back_inserter(stitched_result.a));
        std::copy(result.steer.begin(), result.steer.end(),
                  std::back_inserter(stitched_result.steer));
    }
    stitched_result.x.push_back(partitioned_results.back().x.back());
    stitched_result.y.push_back(partitioned_results.back().y.back());
    stitched_result.phi.push_back(partitioned_results.back().phi.back());
    stitched_result.v.push_back(partitioned_results.back().v.back());
    *result = stitched_result;
    return true;
}

// Hybrid A star的核心过程
// 输入：起点（sx, sy, sphi）、终点（ex, ey, ephi）、地图边界（XYbounds，好像要4个：xmin, xmax, ymin, ymax）、障碍物顶点信息（obstacles_vertices_vec）
// 输出：*result
bool HybridAStar::Plan(
    double sx, double sy, double sphi, double ex, double ey, double ephi,
    const std::vector<double> &XYbounds,
    const std::vector<std::vector<Vec2d>> &obstacles_vertices_vec,
    HybridAStartResult *result)
{
    // clear containers
    // 每次规划，清空之前的缓存数据
    open_set_.clear();
    close_set_.clear();
    open_pq_ = decltype(open_pq_)();
    final_node_ = nullptr;

    std::vector<std::vector<LineSegment2d>> obstacles_linesegments_vec;
    // 构造障碍物轮廓线段容器
    for (const auto &obstacle_vertices : obstacles_vertices_vec)
    {
        size_t vertices_num = obstacle_vertices.size(); // 拿到每个障碍物的边
        std::vector<LineSegment2d> obstacle_linesegments;
        // 我认为这里有错，少构造了一条线段
        for (size_t i = 0; i < vertices_num - 1; ++i) // 依次对该障碍物的每个边读取
        {
            LineSegment2d line_segment = LineSegment2d(obstacle_vertices[i], obstacle_vertices[i + 1]); // 遍历各个顶点得到边，这里应该少了end至start的情况
            obstacle_linesegments.emplace_back(line_segment);
        }
        // 因为我觉得有问题，所以补充了一条线段
        LineSegment2d line_segment = LineSegment2d(obstacle_vertices[0], obstacle_vertices[vertices_num - 1]);
        obstacle_linesegments.emplace_back(line_segment);

        obstacles_linesegments_vec.emplace_back(obstacle_linesegments); // 该障碍物的所有边得到
    }

    obstacles_linesegments_vec_ = std::move(obstacles_linesegments_vec);

    // load XYbounds
    XYbounds_ = XYbounds;
    // load nodes and obstacles
    // 构造规划的起点和终点，并检查其有效性
    start_node_.reset(new Node3d({sx}, {sy}, {sphi}, XYbounds_));
    end_node_.reset(new Node3d({ex}, {ey}, {ephi}, XYbounds_));

    if (!ValidityCheck(start_node_))
    {
        std::cout << "start_node in collision with obstacles";
        return false;
    }
    if (!ValidityCheck(end_node_))
    {
        std::cout << "end_node in collision with obstacles";
        return false;
    }

    // 使用动态规划DP来计算目标点到某点的启发代价（以目标点为DP的起点）
    // 生成graph的同时获得了目标点到图中任一点的cost，后续只需要查表，空间换时间
    grid_a_star_heuristic_generator_->GenerateDpMap(ex, ey, XYbounds_,
                                                    obstacles_linesegments_vec_);

    // std::cout << "map time " << Clock::NowInSeconds() - map_time;
    //  load open set, pq
    open_set_.emplace(start_node_->GetIndex(), start_node_);
    open_pq_.emplace(start_node_->GetIndex(), start_node_->GetCost());

    // Hybrid A* begins
    size_t explored_node_num = 0;
    double heuristic_time = 0.0;
    double rs_time = 0.0;
    while (!open_pq_.empty())
    {
        // take out the lowest cost neighboring node
        const std::string current_id = open_pq_.top().first;
        open_pq_.pop();
        std::shared_ptr<Node3d> current_node = open_set_[current_id];
        // check if an analystic curve could be connected from current
        // configuration to the end configuration without collision. if so, search
        // ends.
        // true：如果生成了一条从当前点到目标点的ReedShepp曲线，就找到了最短路径
        // false：否则，继续Hybrid A*扩展节点
        if (AnalyticExpansion(current_node)) // 用RS曲线试试运气，运气爆棚可以到达终点，则搜索结束
        {
            break;
        }
        close_set_.emplace(current_node->GetIndex(), current_node);
        // 从current_node出发，依次以不同steering，前进arc(对角线长度)
        for (size_t i = 0; i < next_node_num_; ++i)
        {
            // 一个grid内的最后一个路径点叫node，该grid内可以有多个路径点，
            // 该node的next_node一定在相邻的其他grid内
            std::shared_ptr<Node3d> next_node = Next_node_generator(current_node, i);
            // boundary check failure handle
            if (next_node == nullptr)
            {
                continue;
            }
            // check if the node is already in the close set
            if (close_set_.find(next_node->GetIndex()) != close_set_.end())
            {
                continue;
            }
            // collision check, 负责碰撞检测，输入参数节点所连接的、在同一grid内的其他路径点也一起检测了
            if (!ValidityCheck(next_node))
            {
                continue;
            }
            // 从未被探索，进行初始化
            // open_set_其实是close_set_和 open_pq_的合集
            // 每个栅格的index由 x_grid_、y_grid_、phi_grid_共同决定，而不只是x_grid_、y_grid，
            // 不过这里phi_grid_根据 phi_grid_resolution 做了离散化，所以重叠程度还是有的
            if (open_set_.find(next_node->GetIndex()) == open_set_.end())
            {
                explored_node_num++;
                CalculateNodeCost(current_node, next_node);
                open_set_.emplace(next_node->GetIndex(), next_node);
                open_pq_.emplace(next_node->GetIndex(), next_node->GetCost());
            }
        }
    }

    if (final_node_ == nullptr)
    {
        std::cout << "Hybrid A searching return null ptr(open_set ran out)";
        return false;
    }

    if (!GetResult(result))
    {
        std::cout << "GetResult failed";
        return false;
    }

    return true;
}
//

// HybridAStartResult_to_TrajectoryPoint
DiscretizedTrajectory HybridAStar::HybridAStartResult_to_TrajectoryPoint(HybridAStartResult *best_path)
{
    DiscretizedTrajectory Optim_trajectory;
    if (best_path == nullptr)
        return {};

    // std::cout << "x:" << best_path->x.size() << "\n";
    // std::cout << "accumulated_s:" << best_path->accumulated_s.size() << "\n";
    // std::cout << "y:" << best_path->y.size() << "\n";
    // std::cout << "phi:" << best_path->phi.size() << "\n";
    // std::cout << "v:" << best_path->v.size() << "\n";
    // std::cout << "a:" << best_path->a.size() << "\n";

    // for (size_t i = 0; i < best_path->phi.size(); i++)
    // {
    //     std::cout << "phi:" << best_path->phi[i] << "\n";
    // }

    for (int i = 0; i < best_path->x.size() - 1; ++i)
    {
        TrajectoryPoint trajectory_point;
        // 赋值frenet坐标系下的
        trajectory_point.set_relative_time(0);
        trajectory_point.set_s(0);
        trajectory_point.s_d = 0;
        trajectory_point.s_dd = 0;
        trajectory_point.d = 0;
        trajectory_point.d_d = 0;
        trajectory_point.d_dd = 0;
        // 赋值世界坐标系下的
        trajectory_point.set_x(best_path->x[i]);
        trajectory_point.set_y(best_path->y[i]);
        trajectory_point.set_theta(best_path->phi[i]);
        trajectory_point.set_kappa(0);
        trajectory_point.set_v(best_path->v[i]);
        trajectory_point.set_a(best_path->a[i]);

        Optim_trajectory.emplace_back(trajectory_point);
    }

    return Optim_trajectory;
}