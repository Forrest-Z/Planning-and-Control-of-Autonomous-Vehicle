#ifndef PLANNINGTARGET_H
#define PLANNINGTARGET_H
#include <vector>
#include <iostream>
class PlanningTarget
{
public:
  PlanningTarget();
  PlanningTarget(const double speed, const double stop_point, const std::vector<double> &reference_s);
  ~PlanningTarget();
  double cruise_speed() const;
  bool has_stop_point() const;
  void set_stop_point();
  double stop_point() const;

private:
  std::vector<double> reference_s_;
  double speed_;
  double stop_point_;
  bool has;
};
#endif