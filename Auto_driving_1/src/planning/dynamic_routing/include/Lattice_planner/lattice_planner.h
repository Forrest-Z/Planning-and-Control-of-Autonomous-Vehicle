#ifndef LATTICE_PLANNER_H
#define LATTICE_PLANNER_H

#include "lattice_trajectory1d.h"
#include "trajectory_evaluator.h"
#include "trajectory1d_generator.h"
#include "PlanningTarget.h"
#include "path_time_graph.h"
#include "collision_checker.h"
#include "constraint_checker.h"
#include "trajectory_combiner.h"

class LatticePlanner
{
public:
  LatticePlanner();
  ~LatticePlanner() = default;
  DiscretizedTrajectory LatticePlan(
      const InitialConditions &planning_init_point,
      const PlanningTarget &planning_target,
      const std::vector<const Obstacle *> &obstacles,
      const std::vector<double> &accumulated_s,
      const std::vector<ReferencePoint> &reference_points, const bool &lateral_optimization,
      const double &init_relative_time, const double &lon_decision_horizon);

  void visualization_obstacle_trajectory(const std::shared_ptr<PathTimeGraph> &ptr_path_time_graph,
                                         const std::vector<const Obstacle *> &Obstacles);
  void Show_ob_prediction(geometry_msgs::Pose &obstacle_trajectory_, const TrajectoryPoint trajectory_point_);

private:
  TrajectoryCombiner trajectorycombiner;
  ConstraintChecker constraintchecker_;
  ros::Publisher Obstacle_Prediction_;
};

#endif