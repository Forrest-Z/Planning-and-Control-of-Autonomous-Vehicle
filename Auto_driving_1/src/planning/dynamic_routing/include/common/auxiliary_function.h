#ifndef AUXILIARY_FUNCTION_H
#define AUXILIARY_FUNCTION_H

#include <ros/ros.h>
#include <vector>
#include <iostream>
#include "reference_point.h"
#include "CubicSpline2D.h"
#include <geometry_msgs/Pose.h>


class Auxiliary
{
public:
  Auxiliary() = default;
  ~Auxiliary() = default;
  std::pair<double, double> Find_nearest_piont(std::pair<double, double> ObastaclePoint,
                                               std::pair<std::vector<double>, std::vector<double>> reference_path_,
                                               int &index_ob);
  double find_s(CubicSpline2D *csp, double x, double y, double s0, std::vector<double> s);
  double cartesian_to_frenet(std::pair<double, double> ObastaclePoint, std::pair<double, double> rxy, double rtheta);
  geometry_msgs::Pose search_pose;
};
#endif // AUXILIARY_FUNCTION_H
