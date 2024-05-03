#pragma once
#include "Obstacle.h"
#include "ObjectDecision.h"
#include <string>
#include <mutex>

class PathDecision
{
public:
    PathDecision() = default;

    Obstacle *AddObstacle(const Obstacle &obstacle);

    const IndexedList<std::string, Obstacle> &obstacles() const;

    bool AddLateralDecision(const std::string &tag, const std::string &object_id,
                            const ObjectDecisionType &decision);
    bool AddLongitudinalDecision(const std::string &tag,
                                 const std::string &object_id,
                                 const ObjectDecisionType &decision);

    const Obstacle *Find(const std::string &object_id) const;

    Obstacle *Find(const std::string &object_id);

    void SetStBoundary(const std::string &id, const StBoundary &boundary);
    void EraseStBoundaries();
    MainStop main_stop() const { return main_stop_; }
    double stop_reference_line_s() const { return stop_reference_line_s_; }
    bool MergeWithMainStop(const ObjectStop &obj_stop, const std::string &obj_id,
                           const ReferenceLine &ref_line,
                           const SL_Boundary &adc_sl_boundary);

private:
    std::mutex obstacle_mutex_;
    IndexedList<std::string, Obstacle> obstacles_;
    MainStop main_stop_;
    double stop_reference_line_s_ = std::numeric_limits<double>::max();
};