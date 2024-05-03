#ifndef FRENET_OPTIMAL_TRAJECTORY_H
#define FRENET_OPTIMAL_TRAJECTORY_H

#include "FrenetPath.h"
#include "CubicSpline2D.h"
#include "Obstacle.h"
#include <vector>
#include <cmath>
#include <queue>
#include "path_struct.h"
#include "QuarticPolynomial.h"
#include "QuinticPolynomial.h"
#include <chrono>
#include <complex>
#include <ros/ros.h>
#include <algorithm>
#include "FrenetPath.h"

struct CostComparator_for_Frenet : public std::binary_function<const FrenetPath*, const FrenetPath*, bool>
{
  bool operator()(const FrenetPath* left, const FrenetPath* right) const
  {
    return left->J > right->J;
  }
};

class FrenetOptimalTrajectory
{
public:
  FrenetInitialConditions* fot_ic;
  FrenetHyperparameters* fot_hp;
  FrenetOptimalTrajectory();
  ~FrenetOptimalTrajectory();
  bool is_valid_path(FrenetPath& fts);
  double Calculate_curvature(FrenetPath fts);
  void
  calc_frenet_paths(FrenetInitialConditions* fot_ic, FrenetHyperparameters* fot_hp,
                    std::priority_queue<FrenetPath*, std::vector<FrenetPath*>, CostComparator_for_Frenet>& frenet_path,
                    CubicSpline2D* csp, std::vector<double> x, std::vector<double> y);
  void calc_frenet_paths_stop(
      FrenetInitialConditions* fot_ic, FrenetHyperparameters* fot_hp,
      std::priority_queue<FrenetPath*, std::vector<FrenetPath*>, CostComparator_for_Frenet>& frenet_path,
      CubicSpline2D* csp, std::vector<double> x, std::vector<double> y);
  bool to_global_path(FrenetPath& fts, CubicSpline2D* csp, std::vector<double>& x, std::vector<double>& y);
  DiscretizedTrajectory FrenetPath_to_TrajectoryPoint(FrenetPath* best_path);
  constexpr double pi()
  {
    return M_PI;
  }
  double deg2rad(double x)
  {
    return x * pi() / 180;
  }
  double rad2deg(double x)
  {
    return x * 180 / pi();
  }
};

#endif