#include "em_planner.h"
#include "math_utils.h"
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

namespace
{
  constexpr double kPathOptimizationFallbackClost = 2e4;
  constexpr double kSpeedOptimizationFallbackClost = 2e4;
  constexpr double kStraightForwardLineCost = 10.0;

  void ComputeInitFrenetState(const ReferencePoint &matched_point,
                              const TrajectoryPoint &cartesian_state,
                              std::array<double, 3> *ptr_s,
                              std::array<double, 3> *ptr_d)
  {
    CartesianFrenetConverter::cartesian_to_frenet(
        matched_point.accumulated_s_, matched_point.x_, matched_point.y_,
        matched_point.heading_, matched_point.kappa_, matched_point.dkappa_,
        cartesian_state.path_point().x, cartesian_state.path_point().y,
        cartesian_state.v, cartesian_state.a,
        cartesian_state.path_point().theta,
        cartesian_state.path_point().kappa, ptr_s, ptr_d);
  }
} // namespace

EMPlanner::EMPlanner()
{
  ros::NodeHandle n;
  empub1 = n.advertise<nav_msgs::Path>("/xsj/planning/em_planner/em_planner_dp", 10); // 发布测试轨迹
  empub2 = n.advertise<nav_msgs::Path>("/xsj/planning/em_planner/em_planner_qp", 10); // 发布测试轨迹
}

DiscretizedTrajectory EMPlanner::Plan(const TrajectoryPoint &planning_init_point, const ReferenceLine &reference_line,
                                      ReferenceLineInfo &reference_line_info,
                                      std::vector<Obstacle> AllObstacle,
                                      const std::vector<const Obstacle *> &obstacles,
                                      const nav_msgs::Odometry &vehicle_pos,
                                      const double &vehicle_heading, const double &lon_decision_horizon)
{
  DiscretizedTrajectory Optim_trajectory;
  PathData path_data(reference_line);
  SpeedData speed_data;

  SpeedData heuristic_speed_data; // 暂时不知道怎么赋值，是用在动态障碍物的启发速度

  //---------------------------------------------自主车的SL--------------------------------------------------//
  SL_Boundary adc_sl_boundary;
  Vec2d vec_to_center(
      (Config_.front_edge_to_center - Config_.back_edge_to_center) / 2.0,
      (Config_.left_edge_to_center - Config_.right_edge_to_center) / 2.0);
  // realtime vehicle position
  Vec2d vehicle_position(vehicle_pos.pose.pose.position.x, vehicle_pos.pose.pose.position.y);

  Vec2d vehicle_center(vehicle_position + vec_to_center.rotate(vehicle_heading));
  Box2d vehicle_box(vehicle_center, vehicle_heading, Config_.FLAGS_vehicle_length, Config_.FLAGS_vehicle_width);
  if (!reference_line.GetSLBoundary(vehicle_box, &adc_sl_boundary))
  {
    return {};
  }

  //-------------------------------------------障碍物SL和ST获取----------------------------------------------//
  ReferencePoint matched_point = PathMatcher::MatchToPath(reference_line.path_reference(), planning_init_point.path_point().x,
                                                          planning_init_point.path_point().y);
  std::array<double, 3> init_s;
  std::array<double, 3> init_d;

  ComputeInitFrenetState(matched_point, planning_init_point, &init_s, &init_d);
  auto ptr_path_time_graph = std::make_shared<PathTimeGraph>(obstacles, reference_line.path_reference(), init_s[0],
                                                             init_s[0] + 30, // 前瞻多少m
                                                             0.0, Config_.FLAGS_trajectory_time_length, init_d);
  std::vector<const Obstacle *> obstacles_;

  // 过滤地图障碍物
  for (size_t i = 0; i < ptr_path_time_graph->get_path_time_obstacles().size(); i++)
  {
    auto path_time_obstacle = ptr_path_time_graph->get_path_time_obstacles().at(i);

    for (auto &obstacle : AllObstacle)
    {
      //  如果说有ST图的障碍物，才有效
      if (path_time_obstacle.obstacle_id == obstacle.obstacle_id)
      {
        if (obstacle.IsStatic() || obstacle.obstacle_type == 1) // 静态障碍物或者速度低的
        {
          auto static_obs_sl_boundary = ptr_path_time_graph->get_static_obs_sl_boundaries().at(i);

          obstacle.SetSLBoundary(static_obs_sl_boundary);
          obstacle.SetSTBoundary(path_time_obstacle);
          obstacle.SetLateralDecision();
        }
        else
        {
          obstacle.SetSTBoundary(path_time_obstacle);
        }
        obstacles_.push_back(std::move(&obstacle));
      }
    }
  }
  //---------------------------------------EM算法开始----------------------------------------------//

  //---------------------------------------路径规划----------------------------------------------//
  // 1.路径规划 DP
  DpPolyPathOptimizer PathProfile(obstacles_, vehicle_pos);
  if (!PathProfile.Process(heuristic_speed_data, reference_line, planning_init_point, &path_data))
  {
    ROS_INFO("Failed to generate PathProfile");
    return {};
  }
  double total_length_s_ = path_data.Length();

  traj_1.poses.clear();
  traj_1.header.frame_id = Frame_id;
  traj_1.header.stamp = ros::Time::now();
  for (size_t i = 0; i < path_data.Size(); i++)
  {
    geometry_msgs::PoseStamped pose_stamp;
    pose_stamp.pose.position.x = path_data.discretized_path()[i].x;
    pose_stamp.pose.position.y = path_data.discretized_path()[i].y;
    pose_stamp.pose.position.z = 0;
    traj_1.poses.push_back(pose_stamp);
  }
  empub1.publish(traj_1);

  // 2.路径规划 QP
  QpSplinePathOptimizer PathOptimizer; // ActiveSetSpline1dSolver求解器
  if (!PathOptimizer.Process(speed_data, reference_line, planning_init_point, &path_data, obstacles_))
  {
    ROS_INFO("Failed to generate PathOptimizer");
    return {};
  }

  traj_2.poses.clear();
  traj_2.header.frame_id = Frame_id;
  traj_2.header.stamp = ros::Time::now();
  for (size_t i = 0; i < path_data.Size(); i++)
  {
    geometry_msgs::PoseStamped pose_stamp;
    pose_stamp.pose.position.x = path_data.discretized_path()[i].x;
    pose_stamp.pose.position.y = path_data.discretized_path()[i].y;
    pose_stamp.pose.position.z = 0;
    traj_2.poses.push_back(pose_stamp);
  }
  empub2.publish(traj_2);

  //---------------------------------------速度规划----------------------------------------------//
  // 创建速度限制类
  planning::SpeedLimit speed_limits;
  SpeedLimitDecider speed_limit_decider(adc_sl_boundary, reference_line, path_data);
  speed_limit_decider.GetSpeedLimits(obstacles_, &speed_limits);

  // 创建st图对象
  StGraphData st_graph_data;
  st_graph_data.LoadData(ptr_path_time_graph->get_path_time_obstacles(), 0.0, planning_init_point, speed_limits, Config_.default_cruise_speed, total_length_s_, 8.0);

  // 3.速度规划 DP
  PathTimeHeuristicOptimizer SpeedProfile(st_graph_data, obstacles_);
  if (!SpeedProfile.Process(path_data, planning_init_point, &speed_data))
  {
    ROS_INFO("Failed to generate SpeedProfile");
    return {};
  }

  // std::vector<double> spped_v_be, time_t_be;
  // for (size_t i = 0; i < speed_data.size(); i++)
  // {
  //   spped_v_be.emplace_back(speed_data[i].v);
  //   time_t_be.emplace_back(speed_data[i].t);
  // }

  // 4.速度规划 QP
  QpSplineStSpeedOptimizer SpeedOptimizer;
  if (!SpeedOptimizer.Process(adc_sl_boundary, st_graph_data, path_data, planning_init_point, reference_line_info, &speed_data))
  {
    ROS_INFO("Failed to generate SpeedOptimizer");
    return {};
  }

  // plt::clf();
  // std::vector<double> spped_v, time_t;
  // for (size_t i = 0; i < speed_data.size(); i++)
  // {
  //   spped_v.emplace_back(speed_data[i].v);
  //   time_t.emplace_back(speed_data[i].t);
  // }
  // plt::plot(time_t, spped_v, "r--");
  // plt::plot(time_t_be, spped_v_be, "b--");
  // plt::grid(true);
  // plt::ylim(0, 4);
  // plt::xlim(0, 10);
  // plt::pause(0.1);

  // 合并路径规划与速度规划
  if (!CombinePathAndSpeedProfile(
          planning_init_point.relative_time,
          planning_init_point.s, &Optim_trajectory, path_data, speed_data))
  {
    std::string msg("Fail to aggregate planning trajectory.");
    std::cout << msg;
    return {};
  }

  // 更新
  reference_line_info.set_speed_data(speed_data);

  return Optim_trajectory;
}

// 合并路径规划与速度规划
bool EMPlanner::CombinePathAndSpeedProfile(
    const double relative_time, const double start_s,
    DiscretizedTrajectory *ptr_discretized_trajectory, PathData &path_data, SpeedData &speed_data)
{
  // CHECK(ptr_discretized_trajectory != nullptr);
  // use varied resolution to reduce data load but also provide enough data
  // point for control module
  const double kDenseTimeResoltuion = Config_.FLAGS_trajectory_time_min_interval;
  const double kSparseTimeResolution = Config_.FLAGS_trajectory_time_max_interval;
  const double kDenseTimeSec = Config_.FLAGS_trajectory_time_high_density_period;
  if (path_data.discretized_path().size() == 0)
  {
    std::cout << "path data is empty";
    return false;
  }
  if (speed_data.empty())
  {
    std::cout << "speed profile is empty";
    return false;
  }
  for (double cur_rel_time = 0.0; cur_rel_time < speed_data.TotalTime();
       cur_rel_time += (cur_rel_time < kDenseTimeSec ? kDenseTimeResoltuion
                                                     : kSparseTimeResolution))
  {
    SpeedPoint speed_point;
    if (!speed_data.EvaluateByTime(cur_rel_time, &speed_point))
    {
      std::cout << "Fail to get speed point with relative time " << cur_rel_time;
      return false;
    }

    if (speed_point.s > path_data.discretized_path().Length())
    {
      std::cout << "Fail to get speed point with relative time "
                << "\n";
      break;
    }
    PathPoint path_point = path_data.GetPathPointWithPathS(speed_point.s + start_s);

    FrenetFramePoint frenet_point = path_data.GetFrenetFramePointWithPathS(speed_point.s + start_s);

    path_point.set_s(path_point.s);

    TrajectoryPoint trajectory_point;
    trajectory_point.CopyFrom(path_point, frenet_point);

    trajectory_point.set_v(speed_point.v);
    trajectory_point.set_a(speed_point.a);
    trajectory_point.set_relative_time(speed_point.t + relative_time);
    ptr_discretized_trajectory->AppendTrajectoryPoint(trajectory_point);
  }
  return true;
}

// 如果路径规划器或者速度规划器没有找到路径，需要为参考线增加一个cost(20000)，障碍物迫使无人车停车，停车一次增加1000.
bool EMPlanner::PlanOnReferenceLine(const TrajectoryPoint &planning_start_point, const std::vector<const Obstacle *> &obstacles,
                                    ReferenceLineInfo *reference_line_info, const SL_Boundary &adc_sl_boundary, PathData &path_data, SpeedData &speed_data)
{
  if (path_data.Empty())
  {
    GenerateFallbackPathProfile(planning_start_point, adc_sl_boundary,
                                reference_line_info, path_data);
    reference_line_info->AddCost(kPathOptimizationFallbackClost); // 没有找到路径+20000
  }

  if (speed_data.Empty())
  {
    GenerateFallbackSpeedProfile(planning_start_point, speed_data);
    reference_line_info->AddCost(kSpeedOptimizationFallbackClost); // 没有找到路径+20000
  }

  for (const auto *path_obstacle : obstacles)
  {
    if (path_obstacle->IsVirtual())
    {
      continue;
    }
    if (!path_obstacle->IsStatic())
    {
      continue;
    }
    // if (path_obstacle->LongitudinalDecision().has_stop())
    // {
    //   constexpr double kRefrenceLineStaticObsCost = 1e3;
    //   reference_line_info->AddCost(kRefrenceLineStaticObsCost); // 停车一次+1000
    // }
  }
  return true;
}

void EMPlanner::GenerateFallbackPathProfile(const TrajectoryPoint &planning_init_point, const SL_Boundary &adc_sl_boundary,
                                            ReferenceLineInfo *reference_line_info, PathData &path_data)
{
  auto adc_point = planning_init_point; // 自主彻的起点
  double adc_s = adc_sl_boundary.end_s_;
  const double max_s = 150.0;
  const double unit_s = 1.0;

  // projection of adc point onto reference line
  const auto &adc_ref_point =
      reference_line_info->reference_line().GetReferencePoint(0.5 * adc_s);

  // DCHECK(adc_point.has_path_point());
  const double dx = adc_point.path_point().x - adc_ref_point.x_;
  const double dy = adc_point.path_point().y - adc_ref_point.y_;

  std::vector<PathPoint> path_points;
  for (double s = adc_s; s < max_s; s += unit_s)
  {
    const auto &ref_point =
        reference_line_info->reference_line().GetReferencePoint(s);
    PathPoint path_point = MakePathPoint(
        ref_point.x_ + dx, ref_point.y_ + dy, 0.0, ref_point.heading(),
        ref_point.kappa(), ref_point.dkappa(), 0.0);
    path_point.set_s(s);

    path_points.push_back(std::move(path_point));
  }
  path_data.Clear();
  path_data.SetDiscretizedPath(DiscretizedPath(std::move(path_points)));
}

void EMPlanner::GenerateFallbackSpeedProfile(const TrajectoryPoint &planning_init_point, SpeedData &speed_data)
{
  const auto &start_point = planning_init_point;
  speed_data =
      GenerateStopProfileFromPolynomial(start_point.v, start_point.a);
  if (speed_data.empty())
  {
    speed_data = GenerateStopProfile(start_point.v, start_point.a);
  }
}

SpeedData EMPlanner::GenerateStopProfile(const double init_speed, const double init_acc) const
{
  // std::cout << "Slowing down the car.";
  SpeedData speed_data;

  const double kFixedJerk = -1.0;
  const double first_point_acc = std::fmin(0.0, init_acc);

  const double max_t = 3.0;
  const double unit_t = 0.02;

  double pre_s = 0.0;
  const double t_mid = (Config_.FLAGS_slowdown_profile_deceleration - first_point_acc) / kFixedJerk;
  const double s_mid =
      init_speed * t_mid + 0.5 * first_point_acc * t_mid * t_mid + 1.0 / 6.0 * kFixedJerk * t_mid * t_mid * t_mid;
  const double v_mid = init_speed + first_point_acc * t_mid + 0.5 * kFixedJerk * t_mid * t_mid;

  for (double t = 0.0; t < max_t; t += unit_t)
  {
    double s = 0.0;
    double v = 0.0;
    if (t <= t_mid)
    {
      s = std::fmax(pre_s, init_speed * t + 0.5 * first_point_acc * t * t + 1.0 / 6.0 * kFixedJerk * t * t * t);
      v = std::fmax(0.0, init_speed + first_point_acc * t + 0.5 * kFixedJerk * t * t);
      const double a = first_point_acc + kFixedJerk * t;
      speed_data.AppendSpeedPoint(s, t, v, a, 0.0);
      pre_s = s;
    }
    else
    {
      s = std::fmax(pre_s, s_mid + v_mid * (t - t_mid) +
                               0.5 * Config_.FLAGS_slowdown_profile_deceleration * (t - t_mid) * (t - t_mid));
      v = std::fmax(0.0, v_mid + (t - t_mid) * Config_.FLAGS_slowdown_profile_deceleration);
      speed_data.AppendSpeedPoint(s, t, v, Config_.FLAGS_slowdown_profile_deceleration, 0.0);
    }
    pre_s = s;
  }
  return speed_data;
}

SpeedData EMPlanner::GenerateStopProfileFromPolynomial(const double init_speed, const double init_acc) const
{
  // std::cout << "Slowing down the car with polynomial.";
  constexpr double kMaxT = 4.0;
  for (double t = 2.0; t <= kMaxT; t += 0.5)
  {
    for (double s = 0.0; s < 50.0; s += 1.0)
    {
      QuinticPolynomialCurve1d curve(0.0, init_speed, init_acc, s, 0.0, 0.0, t);
      if (!IsValidProfile(curve))
      {
        continue;
      }
      constexpr double kUnitT = 0.02;
      SpeedData speed_data;
      for (double curve_t = 0.0; curve_t <= t; curve_t += kUnitT)
      {
        const double curve_s = curve.Evaluate(0, curve_t);
        const double curve_v = curve.Evaluate(1, curve_t);
        const double curve_a = curve.Evaluate(2, curve_t);
        const double curve_da = curve.Evaluate(3, curve_t);
        speed_data.AppendSpeedPoint(curve_s, curve_t, curve_v, curve_a, curve_da);
      }
      return speed_data;
    }
  }
  return SpeedData();
}

bool EMPlanner::IsValidProfile(const QuinticPolynomialCurve1d &curve) const
{
  for (double evaluate_t = 0.1; evaluate_t <= curve.ParamLength(); evaluate_t += 0.2)
  {
    const double v = curve.Evaluate(1, evaluate_t);
    const double a = curve.Evaluate(2, evaluate_t);
    constexpr double kEpsilon = 1e-3;
    if (v < -kEpsilon || a < -5.0)
    {
      return false;
    }
  }
  return true;
}

PathPoint EMPlanner::MakePathPoint(const double x, const double y, const double z, const double theta,
                                   const double kappa, const double dkappa, const double ddkappa)
{
  PathPoint path_point;
  path_point.set_x(x);
  path_point.set_y(y);
  path_point.set_z(z);
  path_point.set_theta(theta);
  path_point.set_kappa(kappa);
  path_point.set_dkappa(dkappa);
  return path_point;
}

void EMPlanner::CopyFrom(TrajectoryPoint &traj, const InitialConditions &Initial)
{
  traj.CopyFrom(Initial);

  traj.x = Initial.x_init;                         // x position
  traj.y = Initial.y_init;                         // y position
  traj.z = Initial.z_init;                         // z position
  traj.theta = Initial.theta_init;                 // yaw in rad
  traj.kappa = Initial.kappa_init;                 // curvature曲率
  traj.dkappa = Initial.dkappa_init;               // curvature曲率导数
  traj.v = Initial.v_init;                         // Tangential velocity
  traj.a = Initial.a_init;                         // Tangential acceleration
  traj.relative_time = Initial.init_relative_time; // relative_time
  traj.s = Initial.s0;                             // s position along spline

  // 这些不是直接获取的
  traj.d = Initial.d0;      // lateral offset
  traj.d_d = Initial.dd0;   // lateral speed
  traj.d_dd = Initial.ddd0; // lateral acceleration
  traj.s_d = Initial.ds0;   // s speed
  traj.s_dd = Initial.dds0; // s acceleration
  traj.s_ddd = 0;
  traj.d_ddd = 0;
}