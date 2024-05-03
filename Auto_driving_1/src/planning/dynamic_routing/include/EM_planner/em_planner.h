#pragma once
#include "quintic_polynomial_curve1d.h"
#include "dp_poly_path_optimizer.h"
#include "path_time_heuristic_optimizer.h"
#include "qp_spline_path_optimizer.h"
#include "qp_spline_st_speed_optimizer.h"
#include "speed_limit_decider.h"
#include "path_time_graph.h"

class EMPlanner
{
public:
  EMPlanner();

  ~EMPlanner() = default;

  DiscretizedTrajectory Plan(const TrajectoryPoint &planning_init_point, const ReferenceLine &reference_line,
                             ReferenceLineInfo &reference_line_info, std::vector<Obstacle> AllObstacle,
                             const std::vector<const Obstacle *> &obstacles, const nav_msgs::Odometry &vehicle_pos,
                             const double &vehicle_heading, const double &lon_decision_horizon);
                             
  void CopyFrom(TrajectoryPoint &traj, const InitialConditions &Initial);

private:
  bool CombinePathAndSpeedProfile(const double relative_time, const double start_s,
                                  DiscretizedTrajectory *ptr_discretized_trajectory, PathData &path_data, SpeedData &speed_data);

  bool PlanOnReferenceLine(const TrajectoryPoint &planning_start_point, const std::vector<const Obstacle *> &obstacles,
                           ReferenceLineInfo *reference_line_info, const SL_Boundary &adc_sl_boundary, PathData &path_data, SpeedData &speed_data);

  void GenerateFallbackPathProfile(const TrajectoryPoint &planning_init_point, const SL_Boundary &adc_sl_boundary,
                                   ReferenceLineInfo *reference_line_info, PathData &path_data);

  void GenerateFallbackSpeedProfile(const TrajectoryPoint &planning_init_point, SpeedData &speed_data);

  SpeedData GenerateStopProfile(const double init_speed, const double init_acc) const;

  SpeedData GenerateStopProfileFromPolynomial(const double init_speed, const double init_acc) const;

  bool IsValidProfile(const QuinticPolynomialCurve1d &curve) const;

  PathPoint MakePathPoint(const double x, const double y, const double z, const double theta, const double kappa,
                          const double dkappa, const double ddkappa);

  ros::Publisher empub1, empub2;
  geometry_msgs::PoseArray traj_points_;
  nav_msgs::Path traj_1, traj_2;
  std::shared_ptr<PathTimeGraph> ptr_path_time_graph_;
};