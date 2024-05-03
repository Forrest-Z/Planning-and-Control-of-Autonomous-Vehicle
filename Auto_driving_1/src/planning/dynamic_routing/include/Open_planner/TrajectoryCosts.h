/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file
 * Modified function input and used only some functions
 **/
#pragma once

#include "RoadNetwork.h"


class TrajectoryCosts
{
public:
    TrajectoryCosts();
    virtual ~TrajectoryCosts();

    OP_TrajectoryCost DoOneStep(const std::vector<std::vector<std::vector<WayPoint>>> &rollOuts, const std::vector<std::vector<WayPoint>> &totalPaths,
                             const WayPoint &currState, const int &currTrajectoryIndex, const int &currLaneIndex, const PlanningParams &params,
                             const CAR_BASIC_INFO &carInfo, const VehicleState &vehicleState, const std::vector<DetectedObject> &obj_list);

public:
    int m_PrevCostIndex;
    std::vector<OP_TrajectoryCost> m_TrajectoryCosts;
    PlanningParams m_Params;
    PolygonShape m_SafetyBorder;
    std::vector<WayPoint> m_AllContourPoints;
    std::vector<WayPoint> m_CollisionPoints;
    double m_WeightPriority;
    double m_WeightTransition;
    double m_WeightLong;
    double m_WeightLat;
    double m_WeightLaneChange;
    double m_LateralSkipDistance;

private:
    bool ValidateRollOutsInput(const std::vector<std::vector<std::vector<WayPoint>>> &rollOuts);
    std::vector<OP_TrajectoryCost> CalculatePriorityAndLaneChangeCosts(const std::vector<std::vector<WayPoint>> &laneRollOuts, const int &lane_index, const PlanningParams &params);
    void NormalizeCosts(std::vector<OP_TrajectoryCost> &trajectoryCosts);
    void CalculateLateralAndLongitudinalCosts(std::vector<OP_TrajectoryCost> &trajectoryCosts, const std::vector<std::vector<std::vector<WayPoint>>> &rollOuts,
                                              const std::vector<std::vector<WayPoint>> &totalPaths, const WayPoint &currState, const std::vector<WayPoint> &contourPoints,
                                              const PlanningParams &params, const CAR_BASIC_INFO &carInfo, const VehicleState &vehicleState);
    void CalculateTransitionCosts(std::vector<OP_TrajectoryCost> &trajectoryCosts, const int &currTrajectoryIndex, const PlanningParams &params);
};
