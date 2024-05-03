#include "lattice_planner.h"

/*
1.将参考线转变为离散地图点（省去,因为我们的参考线是根据中心线生成的，本身点是离散的，不用再离散化）
2.计算参考线上初始规划点的匹配点
3.根据匹配点计算Frenet帧的初始状态
4.解析决策，得到规划目标
5.分别生成纵向和横向一维轨迹束
6.评价：首先，根据动态约束条件对一维轨迹的可行性进行评价；其次，评估可行的纵向和横向轨迹对，并根据成本进行排序。
7.返回无碰撞的、符合条件的轨迹
*/

namespace
{
  std::vector<PathPoint> ToDiscretizedReferenceLine(const std::vector<ReferencePoint> &ref_points)
  {
    double s = 0.0;
    std::vector<PathPoint> path_points;
    for (const auto &ref_point : ref_points)
    {
      PathPoint path_point;
      path_point.set_x(ref_point.x_);
      path_point.set_y(ref_point.y_);
      path_point.set_theta(ref_point.heading());
      path_point.set_kappa(ref_point.kappa());
      path_point.set_dkappa(ref_point.dkappa());

      if (!path_points.empty())
      {
        double dx = path_point.x - path_points.back().x;
        double dy = path_point.y - path_points.back().y;
        s += std::sqrt(dx * dx + dy * dy);
      }
      path_point.set_s(s);
      path_points.push_back(std::move(path_point));
    }
    return path_points;
  }

  void ComputeInitFrenetState(const ReferencePoint &matched_point,
                              const InitialConditions &cartesian_state,
                              std::array<double, 3> *ptr_s,
                              std::array<double, 3> *ptr_d)
  {
    CartesianFrenetConverter::cartesian_to_frenet(
        matched_point.accumulated_s_, matched_point.x_, matched_point.y_,
        matched_point.heading_, matched_point.kappa_, matched_point.dkappa_,
        cartesian_state.x_init, cartesian_state.y_init,
        cartesian_state.v_init, cartesian_state.a_init,
        cartesian_state.theta_init,
        cartesian_state.kappa_init, ptr_s, ptr_d);
  }

} // namespace

LatticePlanner::LatticePlanner()
{
  ros::NodeHandle nh_;
  Obstacle_Prediction_ = nh_.advertise<geometry_msgs::PoseArray>("/xsj/obstacle/obstacle_prediction", 10); // 发布预测轨迹
}

/*轨迹的生成规划*/
DiscretizedTrajectory LatticePlanner::LatticePlan(
    const InitialConditions &planning_init_point,
    const PlanningTarget &planning_target,
    const std::vector<const Obstacle *> &obstacles,
    const std::vector<double> &accumulated_s,
    const std::vector<ReferencePoint> &reference_points, const bool &lateral_optimization,
    const double &init_relative_time, const double &lon_decision_horizon) // RIO
{
  DiscretizedTrajectory Optim_trajectory;

  // 1. compute the matched point of the init planning point on the reference line.
  ReferencePoint matched_point = PathMatcher::MatchToPath(reference_points, planning_init_point.x_init,
                                                          planning_init_point.y_init);

  // 2. according to the matched point, compute the init state in Frenet frame.
  std::array<double, 3> init_s;
  std::array<double, 3> init_d;
  ComputeInitFrenetState(matched_point, planning_init_point, &init_s, &init_d);

  // 与Apollo不同，我们的前探距离不使用lon_decision_horizon，因为我们的仿真的参考线不长
  auto ptr_path_time_graph = std::make_shared<PathTimeGraph>(obstacles, reference_points, init_s[0],
                                                             init_s[0] + 30, // 前瞻多少m lon_decision_horizon
                                                             0.0, Config_.FLAGS_trajectory_time_length, init_d);

  auto ptr_reference_line = std::make_shared<std::vector<PathPoint>>(ToDiscretizedReferenceLine(reference_points));
  // 通过预测得到障碍物list
  auto ptr_prediction_querier = std::make_shared<PredictionQuerier>(obstacles, ptr_reference_line);
  // 显示预测轨迹
  visualization_obstacle_trajectory(ptr_path_time_graph, obstacles);

  // 3.生成纵向和横向轨迹
  Trajectory1dGenerator trajectory1d_generator(init_s, init_d, ptr_path_time_graph, ptr_prediction_querier);

  std::vector<std::shared_ptr<Curve1d>> lon_trajectory1d_bundle;
  std::vector<std::shared_ptr<Curve1d>> lat_trajectory1d_bundle;
  trajectory1d_generator.GenerateTrajectoryBundles(planning_target, &lon_trajectory1d_bundle, &lat_trajectory1d_bundle);

  // 打印轨迹数
  //  std::cout << "lon_trajectory1d_bundle:" << lon_trajectory1d_bundle.size() << "\n";
  //  std::cout << "lat_trajectory1d_bundle:" << lat_trajectory1d_bundle.size() << "\n";

  // 4.计算每条轨迹的代价,并得出优先级队列
  TrajectoryEvaluator trajectory_evaluator(planning_target, lon_trajectory1d_bundle, lat_trajectory1d_bundle,
                                           init_s, ptr_path_time_graph, reference_points);

  // 5.轨迹拼接和最后的筛选
  while (trajectory_evaluator.has_more_trajectory_pairs())
  {
    double trajectory_pair_cost = trajectory_evaluator.top_trajectory_pair_cost();
    auto trajectory_pair = trajectory_evaluator.next_top_trajectory_pair();
    // combine two 1d trajectories to one 2d trajectory
    auto combined_trajectory = trajectorycombiner.Combine(accumulated_s, *trajectory_pair.first, *trajectory_pair.second,
                                                          reference_points, init_relative_time);

    // 采样时候才调用，二次规划不用
    if (lateral_optimization == false)
    {
      // check longitudinal and lateral acceleration
      // considering trajectory curvatures
      auto result = constraintchecker_.ValidTrajectory(combined_trajectory);
      if (result != ConstraintChecker::Result::VALID)
      {
        switch (result)
        {
        case ConstraintChecker::Result::LON_VELOCITY_OUT_OF_BOUND:
          break;
        case ConstraintChecker::Result::LON_ACCELERATION_OUT_OF_BOUND:
          break;
        case ConstraintChecker::Result::LON_JERK_OUT_OF_BOUND:
          break;
        case ConstraintChecker::Result::CURVATURE_OUT_OF_BOUND:
          break;
        case ConstraintChecker::Result::LAT_ACCELERATION_OUT_OF_BOUND:
          break;
        case ConstraintChecker::Result::LAT_JERK_OUT_OF_BOUND:
          break;
        case ConstraintChecker::Result::VALID:
        default:
          // Intentional empty
          break;
        }
        continue;
      }
      // Get instance of collision checker and constraint checker
      CollisionChecker collision_checker(obstacles, init_s[0], init_d[0], reference_points, ptr_path_time_graph);
      // 碰撞检测
      if (collision_checker.InCollision(combined_trajectory))
      {
        continue;
      }
    }
    Optim_trajectory = std::move(combined_trajectory);
    break;
  }

  return Optim_trajectory;
}

// 显示预测轨迹
void LatticePlanner::visualization_obstacle_trajectory(const std::shared_ptr<PathTimeGraph> &ptr_path_time_graph,
                                                       const std::vector<const Obstacle *> &Obstacles)
{
  geometry_msgs::PoseArray obstacle_trajectories_;
  obstacle_trajectories_.header.frame_id = Frame_id;
  obstacle_trajectories_.header.stamp = ros::Time::now();

  for (auto &obstacle : Obstacles)
  {
    if (!ptr_path_time_graph->IsObstacleInGraph(obstacle->obstacle_id))
    {
      continue;
    }
    std::vector<TrajectoryPoint> trajectory_points_ = obstacle->Trajectory().trajectory_point();
    // std::cout << trajectory_points_.size() << "\n";
    if (trajectory_points_.size() > 0)
    {
      for (size_t i = 0; i < trajectory_points_.size(); i++)
      {
        geometry_msgs::Pose obstacle_trajectory;
        Show_ob_prediction(obstacle_trajectory, trajectory_points_[i]);
        obstacle_trajectories_.poses.push_back(obstacle_trajectory);
      }
    }
  }
  Obstacle_Prediction_.publish(obstacle_trajectories_);
}

void LatticePlanner::Show_ob_prediction(geometry_msgs::Pose &obstacle_trajectory_, const TrajectoryPoint trajectory_point_)
{
  obstacle_trajectory_.orientation = tf::createQuaternionMsgFromYaw(trajectory_point_.theta);
  obstacle_trajectory_.position.x = trajectory_point_.x;
  obstacle_trajectory_.position.y = trajectory_point_.y;
  obstacle_trajectory_.position.z = 0;
}