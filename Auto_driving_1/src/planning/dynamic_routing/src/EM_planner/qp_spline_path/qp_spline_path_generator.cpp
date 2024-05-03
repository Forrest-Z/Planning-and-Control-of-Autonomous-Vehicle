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
#include "qp_spline_path_generator.h"
#include <algorithm>
#include <utility>
#include <iostream>
#include "macros.h"
#include "cartesian_frenet_conversion.h"
#include "math_utils.h"
#include "point_factory.h"

namespace
{
  double GetLaneChangeLateralShift(const double v)
  {
    const double l0 = 2.0;      // shift at v = 0 m/s
    const double v_ref = 20.11; // reference speed: 45mph = 20.11 m/s
    const double l_ref = 1.4;
    const double b = l0;
    const double a = (l_ref - b) / v_ref;
    return a * v + b;
  }
} // namespace

QpSplinePathGenerator::QpSplinePathGenerator(Spline1dSolver *spline_solver, const ReferenceLine &reference_line)
    : spline_solver_(spline_solver), reference_line_(reference_line)
{
}

void QpSplinePathGenerator::SetChangeLane(bool is_change_lane_path)
{
  is_change_lane_path_ = is_change_lane_path;
}

/*
QpSplinePathGenerator::Generate()中，主要有以下关键步骤：
  初始化样条曲线InitSpline()，
  初始化用来计算自车轨迹横向约束的类QpFrenetFrame::Init()，
  添加约束AddConstraint()，
  添加目标函数AddKernel()，
  优化问题求解器Solve()，
  最后将求得的轨迹点封装成DiscretizedPath.
*/
bool QpSplinePathGenerator::Generate(const std::vector<const Obstacle *> &obstacles, const SpeedData &speed_data,
                                     const TrajectoryPoint &init_point, const double boundary_extension,
                                     bool is_final_attempt, PathData *const path_data)
{
  init_trajectory_point_ = init_point;

  const auto &path_data_history = path_data->path_data_history();
  if (!path_data_history.empty())
  {
    last_discretized_path_ = &path_data_history.back().first;
  }

  init_frenet_point_ = reference_line_.GetFrenetPoint(init_point.path_point());
  if (is_change_lane_path_)
  {
    ref_l_ = init_frenet_point_.d;
  }
  double start_s = init_frenet_point_.s;

  double kDefaultPathLength = Config_.PathLength; 
  double end_s =
      std::fmin(init_frenet_point_.s + std::fmax(kDefaultPathLength, init_trajectory_point_.v * Config_.look_forward_time_sec),
                reference_line_.Length());

  double kMinPathLength = 1.0e-6;
  if (start_s + kMinPathLength > end_s)
  {
    std::cout << "Path length is too small. Path start_s: " << start_s << ", end_s: " << end_s;
    return false;
  }

  if (!InitSpline(start_s, end_s)) // InitSpline()主要完成对spline segment和纵向s采样点的初始化。
  {
    std::cout << "Init smoothing spline failed with (" << start_s << ",  end_s " << end_s;
    return false;
  }

  QpFrenetFrame qp_frenet_frame(reference_line_, speed_data, init_frenet_point_, Config_.time_resolution, evaluated_s_);

  if (!qp_frenet_frame.Init(obstacles))
  {
    std::cout << "Fail to initialize qp frenet frame";
    return false;
  }

  if (!AddConstraint(qp_frenet_frame, boundary_extension))
  {
    std::cout << "Fail to setup pss path constraint.";
    return false;
  }

  AddKernel();

  bool is_solved = Solve();

  if (!is_solved && !is_final_attempt)
  {
    std::cout << "Fail to solve qp_spline_path. Use reference line as qp_path output.";
    return false;
  }

  // extract data
  const Spline1d &spline = spline_solver_->spline();
  std::vector<PathPoint> path_points;

  ReferencePoint ref_point = reference_line_.GetReferencePoint(start_s);
  Vec2d xy_point = CartesianFrenetConverter::CalculateCartesianPoint(
      ref_point.heading_, Vec2d(ref_point.x_, ref_point.y_), init_frenet_point_.d);

  // std::cout << "x:" << init_point.path_point().x << ","
  //           << "y:" << init_point.path_point().y << "\n";
  const auto xy_diff = xy_point - Vec2d(init_point.path_point().x, init_point.path_point().y);

  double s_resolution = (end_s - start_s) / Config_.num_output;
  constexpr double kEpsilon = 1e-6;

  // std::cout << "start_s:" << start_s << ","
  //           << "end_s:" << end_s << ","
  //           << "s_resolution:" << s_resolution << "\n";

  for (double s = start_s; s + kEpsilon < end_s; s += s_resolution)
  {
    double l = init_frenet_point_.d;
    if (is_solved)
    {
      l = spline(s) + ref_l_;
    }
    double dl = 0.0;
    double ddl = 0.0;

    if (is_solved)
    {
      dl = spline.Derivative(s);
      ddl = spline.SecondOrderDerivative(s);
    }
    ReferencePoint ref_point = reference_line_.GetReferencePoint(s);
    Vec2d curr_xy_point = CartesianFrenetConverter::CalculateCartesianPoint(ref_point.heading(), Vec2d(ref_point.x_, ref_point.y_), l) - xy_diff;
    double theta = CartesianFrenetConverter::CalculateTheta(ref_point.heading(), ref_point.kappa(), l, dl);
    double kappa = CartesianFrenetConverter::CalculateKappa(ref_point.kappa(), ref_point.dkappa(), l, dl, ddl);

    double dkappa = 0.0;
    double s_new = 0.0;
    if (!path_points.empty())
    {
      Vec2d last = util::PointFactory::ToVec2d(path_points.back());
      const double distance = (last - curr_xy_point).Length();
      s_new = path_points.back().s + distance;
      if (distance > 1e-4)
        dkappa = (kappa - path_points.back().kappa) / distance;
    }
    PathPoint path_point = util::PointFactory::ToPathPoint(curr_xy_point.x(), curr_xy_point.y(), 0.0, s_new, theta, kappa, dkappa);

    if (path_point.s > end_s)
    {
      break;
    }
    path_points.emplace_back(std::move(path_point));
  }

  path_data->Clear();
  path_data->SetDiscretizedPath(DiscretizedPath(path_points));
  return true;
}

// InitSpline()主要完成对spline segment和纵向s采样点的初始化。
bool QpSplinePathGenerator::InitSpline(const double start_s, const double end_s)
{
  uint32_t number_of_spline = static_cast<uint32_t>((end_s - start_s) / Config_.max_spline_length + 1.0);
  number_of_spline = std::max(1u, number_of_spline);
  common::math::uniform_slice(start_s, end_s, number_of_spline, &knots_);

  // spawn a new spline generator
  spline_solver_->Reset(knots_, Config_.spline_order);

  // set evaluated_s_
  uint32_t constraint_num = static_cast<uint32_t>((end_s - start_s) / Config_.max_constraint_interval + 1.0);
  common::math::uniform_slice(start_s, end_s, constraint_num - 1, &evaluated_s_);
  return (knots_.size() > 1) && !evaluated_s_.empty();
}

bool QpSplinePathGenerator::AddConstraint(const QpFrenetFrame &qp_frenet_frame, const double boundary_extension)
{
  Spline1dConstraint *spline_constraint = spline_solver_->mutable_spline_constraint();

  const int dim = static_cast<int>((knots_.size() - 1) * (Config_.spline_order + 1));
  constexpr double param_range = 1e-4;
  // 0、添加spline多项式系数中的最高次项系数取值范围约束，不等式约束
  // 不太有把握，也可能是最低次项系数
  // 综合起来，限定了 -param_range <= f <= param_range
  for (int i = Config_.spline_order; i < dim; i += Config_.spline_order + 1)
  {
    Eigen::MatrixXd mat = Eigen::MatrixXd::Zero(1, dim);
    Eigen::MatrixXd bd = Eigen::MatrixXd::Zero(1, 1);
    // 对5次多项式，有[0,0,0,0,0,-1] * [a, b, c, d, e, f].T >= -param_range
    // 即 f <= param_range
    mat(0, i) = -1;
    bd(0, 0) = -param_range;
    spline_constraint->AddInequalityConstraint(mat, bd);
    //[0,0,0,0,0,1] * [a, b, c, d, e, f].T >= -param_range
    // 即 f >= -param_range
    mat(0, i) = 1;
    bd(0, 0) = -param_range;
    spline_constraint->AddInequalityConstraint(mat, bd);
  }

  // add init status constraint, equality constraint
  // 1、添加起始点处函数值范围约束，不等式约束
  // 限定了min_d <= d(s) <= max_d
  const double kBoundaryEpsilon = 1e-4;
  spline_constraint->AddPointConstraintInRange(init_frenet_point_.s, init_frenet_point_.d - ref_l_, kBoundaryEpsilon);
  // 2、添加起始点处一阶导范围约束，不等式约束
  // 限定了min_d' <= d'(s) <= max_d'
  spline_constraint->AddPointDerivativeConstraintInRange(init_frenet_point_.s, init_frenet_point_.d_d,
                                                         kBoundaryEpsilon);
  // path二阶导其实是曲线的曲率kappa，若过U形弯速度快，则不对kappa添加约束
  if (init_trajectory_point_.v > Config_.uturn_speed_limit)
  {
    // 3、添加起始点处二阶导范围约束，不等式约束
    // 限定了min_d'' <= d''(s) <= max_d''
    spline_constraint->AddPointSecondDerivativeConstraintInRange(init_frenet_point_.s, init_frenet_point_.d_dd,
                                                                 kBoundaryEpsilon);
  }

  // add end point constraint, equality constraint
  double lat_shift = -ref_l_;
  // lat_shift看不懂，为什么要与ref_l_反向？
  // 推测：换道时有参考线切换，自车横向坐标在目标车道参考线坐标系下是ref_l_，目标点应该在目标车道参考线上采样，
  // 所以目标点在当前车道参考线坐标系下横向坐标大概是fabs(ref_l_)，而正负一定相反
  // 如果不变道，ref_l_=0，如果变道，ref_l_=init_frenet_point_.l();

  if (is_change_lane_path_)
  {
    double lane_change_lateral_shift = GetLaneChangeLateralShift(init_trajectory_point_.v);
    lat_shift = std::copysign(std::fmin(std::fabs(ref_l_), lane_change_lateral_shift), -ref_l_);
  }

  // std::cout << "lat_shift = " << lat_shift;
  const double kEndPointBoundaryEpsilon = 1e-2;
  constexpr double kReservedDistance = 20.0;
  const double target_s = std::fmin(Config_.point_constraint_s_position, kReservedDistance + init_frenet_point_.s +
                                                                             init_trajectory_point_.v * Config_.look_forward_time_sec);
  // 4、添加终止点处函数值范围约束，不等式约束
  // 为什么此处使用target_s，而不使用evaluated_s_.back()，因为终点指所有路径的终点
  spline_constraint->AddPointConstraintInRange(target_s, lat_shift, kEndPointBoundaryEpsilon);

  // 5、添加终止点处一阶导范围约束，不等式约束
  // 如果是变道，则不约束一阶导即方向角，如果不是变道，则目标点的方向=0，即沿s轴
  if (!is_change_lane_path_)
  {
    spline_constraint->AddPointDerivativeConstraintInRange(evaluated_s_.back(), 0.0, kEndPointBoundaryEpsilon);
  }

  // dl_bound是0.1，arctan(0.1)=5.71度，这角度会不会太小了？
  // add first derivative bound to improve lane change smoothness
  std::vector<double> dl_lower_bound(evaluated_s_.size(), -Config_.FLAGS_dl_bound);
  std::vector<double> dl_upper_bound(evaluated_s_.size(), Config_.FLAGS_dl_bound);

  // 6、添加各采样点朝向角范围约束，不等式约束
  if (!spline_constraint->AddDerivativeBoundary(evaluated_s_, dl_lower_bound, dl_upper_bound))
  {
    // std::cout << "Fail to add second derivative boundary.";
    return false;
  }

  // kappa bound is based on the inequality:
  // kappa = d(phi)/ds <= d(phi)/dx = d2y/dx2
  std::vector<double> kappa_lower_bound(evaluated_s_.size(), -Config_.FLAGS_kappa_bound);
  std::vector<double> kappa_upper_bound(evaluated_s_.size(), Config_.FLAGS_kappa_bound);
  // 7、添加各采样点曲率Kappa范围约束，不等式约束
  if (!spline_constraint->AddSecondDerivativeBoundary(evaluated_s_, kappa_lower_bound, kappa_upper_bound))
  {
    // std::cout << "Fail to add second derivative boundary.";
    return false;
  }

  // dkappa = d(kappa) / ds <= d3y/dx3
  std::vector<double> dkappa_lower_bound(evaluated_s_.size(), -Config_.FLAGS_dkappa_bound);
  std::vector<double> dkappa_upper_bound(evaluated_s_.size(), Config_.FLAGS_dkappa_bound);
  if (!spline_constraint->AddThirdDerivativeBoundary(evaluated_s_, dkappa_lower_bound, dkappa_upper_bound))
  {
    // std::cout << "Fail to add third derivative boundary.";
    return false;
  }

  // add map bound constraint
  double lateral_buf = boundary_extension;
  if (is_change_lane_path_)
  {
    lateral_buf = Config_.cross_lane_lateral_extension;
  }
  std::vector<double> boundary_low;
  std::vector<double> boundary_high;

  // 综合各种横向区间约束，求取最终的横向约束
  for (uint32_t i = 0; i < evaluated_s_.size(); ++i)
  {
    auto road_boundary = qp_frenet_frame.GetMapBound().at(i);                  // 道路边界
    auto static_obs_boundary = qp_frenet_frame.GetStaticObstacleBound().at(i); // 静态障碍物边界
    // auto dynamic_obs_boundary = qp_frenet_frame.GetDynamicObstacleBound().at(i);  //动态障碍物边界,暂不考虑

    if (evaluated_s_.at(i) - evaluated_s_.front() < Config_.cross_lane_lateral_extension)
    {
      // 右边界
      road_boundary.first = std::fmin(road_boundary.first, init_frenet_point_.d - lateral_buf);
      // 左边界
      road_boundary.second = std::fmax(road_boundary.second, init_frenet_point_.d + lateral_buf);
    }
    boundary_low.emplace_back(
        common::math::MaxElement(std::vector<double>{road_boundary.first, static_obs_boundary.first}));
    boundary_high.emplace_back(
        common::math::MinElement(std::vector<double>{road_boundary.second, static_obs_boundary.second}));

    // boundary_low.emplace_back(common::math::MaxElement(
    //     std::vector<double>{ road_boundary.first, static_obs_boundary.first, dynamic_obs_boundary.first }));
    // boundary_high.emplace_back(common::math::MinElement(
    //     std::vector<double>{ road_boundary.second, static_obs_boundary.second, dynamic_obs_boundary.second }));

    //  std::cout << "s[" << evaluated_s_[i] << "] boundary_low[" << boundary_low.back() << "] boundary_high["
    //   << boundary_high.back() << "] road_boundary_low[" << road_boundary.first << "] road_boundary_high["
    //   << road_boundary.second << "] static_obs_boundary_low[" << static_obs_boundary.first
    //   << "] static_obs_boundary_high[" << static_obs_boundary.second << "] dynamic_obs_boundary_low["
    //   << dynamic_obs_boundary.first << "] dynamic_obs_boundary_high[" << dynamic_obs_boundary.second << "].";
  }
  // 不等式约束
  const double start_l = ref_l_;
  // 求横向相对坐标
  std::for_each(boundary_low.begin(), boundary_low.end(), [start_l](double &d)
                { d -= start_l; });
  std::for_each(boundary_high.begin(), boundary_high.end(), [start_l](double &d)
                { d -= start_l; });
  // 9、添加各采样点函数值（横向坐标）范围约束，不等式约束
  if (!spline_constraint->AddBoundary(evaluated_s_, boundary_low, boundary_high))
  {
    // std::cout << "Add boundary constraint failed";
    return false;
  }

  // add spline joint third derivative constraint
  // 10、添加各采样点处0~3阶导连续约束，等式约束，就是平滑约束
  if (knots_.size() >= 3 && !spline_constraint->AddThirdDerivativeSmoothConstraint())
  {
    // std::cout << "Add spline joint derivative constraint failed!";
    return false;
  }
  return true;
}

void QpSplinePathGenerator::AddHistoryPathKernel()
{
  if (last_discretized_path_ == nullptr)
  {
    return;
  }

  PathData last_path_data(reference_line_);
  last_path_data.SetDiscretizedPath(*last_discretized_path_);
  // std::cout << "---------------history_path_-------------------"
  //           << "\n";
  // for (size_t i = 0; i < last_path_data.Size(); i++)
  // {
  //   std::cout << "(" << last_path_data.discretized_path()[i].x << "," << last_path_data.discretized_path()[i].y <<
  //   ")"
  //             << "\n";
  // }
  // for (size_t i = 0; i < last_path_data.Size(); i++)
  // {
  //   std::cout << "(" << last_path_data.frenet_frame_path().at(i).s << "," <<
  //   last_path_data.frenet_frame_path().at(i).d
  //             << ")"
  //             << ","
  //             << "ref_l_:" << ref_l_ << "\n";
  // }
  std::vector<double> history_s;
  std::vector<double> histroy_l;
  // std::cout << "last_path_data.frenet_frame_path().size():" << last_path_data.frenet_frame_path().size() << "\n";
  // std::cout << "ref_l_:" << ref_l_ << "\n";

  for (size_t i = 0; i < last_path_data.frenet_frame_path().size(); ++i)
  {
    const auto p = last_path_data.frenet_frame_path().at(i);
    history_s.push_back(p.s);
    histroy_l.push_back(p.d - ref_l_);
  }

  Spline1dKernel *spline_kernel = spline_solver_->mutable_spline_kernel();
  spline_kernel->AddReferenceLineKernelMatrix(history_s, histroy_l, Config_.history_path_weight);
}

void QpSplinePathGenerator::AddKernel()
{
  Spline1dKernel *spline_kernel = spline_solver_->mutable_spline_kernel();

  if (init_trajectory_point_.v < Config_.uturn_speed_limit && !is_change_lane_path_ && Config_.reference_line_weight > 0.0)
  {
    // 只有在!is_change_lane_path_的情况才会进入，而!is_change_lane_path_会导致ref_l_=0
    std::vector<double> ref_l(evaluated_s_.size(), -ref_l_);
    // 添加和参考线相关的kernel，推测应该是使轨迹尽可能贴近参考线，但我在代码中没有理解这一点
    // 而且每次ref_l元素全是0，会导致函数内offset_全是0，意义何在？
    // 参考线或下面的历史轨迹，可以是DP输出的粗糙规划结果
    spline_kernel->AddReferenceLineKernelMatrix(evaluated_s_, ref_l, Config_.reference_line_weight);
  }

  if (Config_.history_path_weight > 0.0)
  {
    // 添加和历史轨迹相关的kernel，推测应该是使轨迹尽可能近似延续之前的轨迹，但我在代码中没有理解这一点
    AddHistoryPathKernel();
  }

  if (Config_.regularization_weight > 0.0)
  {
    spline_kernel->AddRegularization(Config_.regularization_weight);
  }

  if (Config_.derivative_weight > 0.0)
  {
    spline_kernel->AddDerivativeKernelMatrix(Config_.derivative_weight);
    if (std::fabs(init_frenet_point_.d) > Config_.lane_change_mid_l)
    {
      spline_kernel->AddDerivativeKernelMatrixForSplineK(0, Config_.first_spline_weight_factor * Config_.derivative_weight);
    }
  }

  if (Config_.second_derivative_weight > 0.0)
  {
    spline_kernel->AddSecondOrderDerivativeMatrix(Config_.second_derivative_weight);
    if (std::fabs(init_frenet_point_.d) > Config_.lane_change_mid_l)
    {
      spline_kernel->AddSecondOrderDerivativeMatrixForSplineK(0, Config_.first_spline_weight_factor * Config_.second_derivative_weight);
    }
  }

  if (Config_.third_derivative_weight > 0.0)
  {
    spline_kernel->AddThirdOrderDerivativeMatrix(Config_.third_derivative_weight);
    if (std::fabs(init_frenet_point_.d) > Config_.lane_change_mid_l)
    {
      spline_kernel->AddThirdOrderDerivativeMatrixForSplineK(0, Config_.first_spline_weight_factor * Config_.third_derivative_weight);
    }
  }
}

bool QpSplinePathGenerator::Solve()
{
  if (!spline_solver_->Solve())
  {
    // for (size_t i = 0; i < knots_.size(); ++i)
    // {
    //   // std::cout << "knots_[" << i << "]: " << knots_[i];
    // }
    // for (size_t i = 0; i < evaluated_s_.size(); ++i)
    // {
    //   // std::cout << "evaluated_s_[" << i << "]: " << evaluated_s_[i];
    // }
    // std::cout << "Could not solve the qp problem in spline path generator.";
    return false;
  }
  return true;
}