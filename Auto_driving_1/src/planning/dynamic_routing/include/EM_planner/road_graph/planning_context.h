#pragma once
#include <vector>
#include <string>

//字面翻译理解
/*
更改车道状态
人行横道状态
征求意见
重新路由状态
通行权状态
侧传状态
停止标志状态
目的地状态
靠边停车状态
*/
enum class PlanningStatus
{
  ChangeLaneStatus = 1,
  CrosswalkStatus = 2,
  EngageAdvice = 3,
  ReroutingStatus = 4,
  RightOfWayStatus = 5,
  SidePassStatus = 6,
  StopSignStatus = 7,
  DestinationStatus = 8,
  PullOverStatus = 9,
};

struct PullOverStatus
{
  enum class Status
  {
    UNKNOWN = 1,
    IN_OPERATION = 2,
    DONE = 3,
    DISABLED = 4,
  };
  bool in_pull_over = true;  // [default = false]
  int status = 2;
  int inlane_dest_point = 3;
  int start_point = 4;
  int stop_point = 5;
  double stop_point_heading = 6;
  int reason = 7;
  double status_set_time = 8;
};

struct ProceedWithCautionSpeedParam
{
  bool is_fixed_distance = false;
  double distance = 5.0;  // m
};
struct PathOverlap
{
  PathOverlap() = default;
  PathOverlap(std::string object_id, const double start_s, const double end_s)
    : object_id(std::move(object_id)), start_s(start_s), end_s(end_s)
  {
  }

  std::string object_id;
  double start_s = 0.0;
  double end_s = 0.0;

  std::string DebugString() const;
};
class PlanningContext
{
public:
  PlanningContext();
  ~PlanningContext() = default;
  // scenario context
  struct ScenarioInfo
  {
    PathOverlap next_stop_sign_overlap;
    PathOverlap next_traffic_light_overlap;
    PathOverlap next_crosswalk_overlap;
    // still in the scenario for this overlap, but stop already done
    // => no stop fence from decider_rule_based_stop task
    std::string stop_done_overlap_id;
    ProceedWithCautionSpeedParam proceed_with_caution_speed;
    std::vector<std::string> stop_sign_wait_for_obstacles;
    std::vector<std::string> crosswalk_wait_for_obstacles;
  };

  static void Clear();
  static void Init();

  //规划的类型和场景的类型，不移植先
  static const PlanningStatus& Planningstatus()
  {
    return planning_status_;
  }

  static PlanningStatus* MutablePlanningStatus()
  {
    return &planning_status_;
  }

  static ScenarioInfo* GetScenarioInfo()
  {
    return &scenario_info_;
  }

private:
  static PlanningStatus planning_status_;  //规划的类型
  static ScenarioInfo scenario_info_;
};
