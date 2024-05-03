#pragma once

#include <memory>
#include <string>
#include <vector>
#include "active_set_spline_1d_solver.h"
#include "qp_spline_path_generator.h"

class QpSplinePathOptimizer
{
public:
  QpSplinePathOptimizer();
  ~QpSplinePathOptimizer() = default;

  bool Process(const SpeedData& speed_data, const ReferenceLine& reference_line, const TrajectoryPoint& init_point,
               PathData* const path_data, const std::vector<const Obstacle*>& obstacles);

private:
  std::vector<double> init_knots_;
  std::unique_ptr<Spline1dSolver> spline_solver_;
};