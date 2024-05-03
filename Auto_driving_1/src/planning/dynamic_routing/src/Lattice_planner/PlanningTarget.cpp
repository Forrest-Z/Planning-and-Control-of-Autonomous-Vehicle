#include "PlanningTarget.h"
PlanningTarget::PlanningTarget()
{
}
PlanningTarget::~PlanningTarget()
{
}
PlanningTarget::PlanningTarget(const double speed, const double stop_point, const std::vector<double> &reference_s)
    : speed_(speed), stop_point_(stop_point), reference_s_(reference_s)
{
  has = false;
}
double PlanningTarget::cruise_speed() const
{
  return speed_;
}
double PlanningTarget::stop_point() const
{
  return stop_point_ > reference_s_.back() ? reference_s_.back() : stop_point_;
}
bool PlanningTarget::has_stop_point() const
{
  return has;
}

void PlanningTarget::set_stop_point()
{
  has = true;
}