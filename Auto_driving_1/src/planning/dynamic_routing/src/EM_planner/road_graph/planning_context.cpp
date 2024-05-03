#include "planning_context.h"
PlanningContext::ScenarioInfo PlanningContext::scenario_info_;

PlanningContext::PlanningContext()
{
}

void PlanningContext::Init()
{
}

void PlanningContext::Clear()
{
  // planning_status_.Clear();
  scenario_info_ = {};
}