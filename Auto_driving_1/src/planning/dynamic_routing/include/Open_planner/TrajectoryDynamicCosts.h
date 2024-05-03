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

using namespace std;


class TrajectoryDynamicCosts
{
public:
    TrajectoryDynamicCosts();
    virtual ~TrajectoryDynamicCosts();

    OP_TrajectoryCost DoOneStep(const vector<vector<vector<WayPoint>>> &rollOuts, const vector<vector<WayPoint>> &totalPaths,
                                const WayPoint &currState, const int &currTrajectoryIndex, const int &currLaneIndex, const PlanningParams &params,
                                const CAR_BASIC_INFO &carInfo, const VehicleState &vehicleState, const std::vector<DetectedObject> &obj_list);

    OP_TrajectoryCost DoOneStepStatic(const vector<vector<WayPoint>> &rollOuts, const vector<WayPoint> &totalPaths,
                                      const WayPoint &currState, const PlanningParams &params, const CAR_BASIC_INFO &carInfo, const VehicleState &vehicleState,
                                      const std::vector<DetectedObject> &obj_list, const int &iCurrentIndex = -1);

    OP_TrajectoryCost DoOneStepDynamic(const vector<vector<WayPoint>> &rollOuts, const vector<WayPoint> &totalPaths,
                                       const WayPoint &currState, const PlanningParams &params, const CAR_BASIC_INFO &carInfo, const VehicleState &vehicleState,
                                       const std::vector<DetectedObject> &obj_list, const int &iCurrentIndex = -1);

public:
    int m_PrevCostIndex;
    int m_PrevIndex;
    vector<OP_TrajectoryCost> m_TrajectoryCosts;
    PlanningParams m_Params;
    PolygonShape m_SafetyBorder;
    vector<WayPoint> m_AllContourPoints;
    vector<WayPoint> m_CollisionPoints;
    double m_WeightPriority;
    double m_WeightTransition;
    double m_WeightLong;
    double m_WeightLat;
    double m_WeightLaneChange;
    double m_LateralSkipDistance;
    double m_CollisionTimeDiff;

private:
    bool ValidateRollOutsInput(const vector<vector<vector<WayPoint>>> &rollOuts);
    vector<OP_TrajectoryCost> CalculatePriorityAndLaneChangeCosts(const vector<vector<WayPoint>> &laneRollOuts, const int &lane_index, const PlanningParams &params);
    void NormalizeCosts(vector<OP_TrajectoryCost> &trajectoryCosts);
    void CalculateLateralAndLongitudinalCosts(vector<OP_TrajectoryCost> &trajectoryCosts, const vector<vector<vector<WayPoint>>> &rollOuts, const vector<vector<WayPoint>> &totalPaths, const WayPoint &currState, const vector<WayPoint> &contourPoints, const PlanningParams &params, const CAR_BASIC_INFO &carInfo, const VehicleState &vehicleState);
    void CalculateLateralAndLongitudinalCostsStatic(vector<OP_TrajectoryCost> &trajectoryCosts, const vector<vector<WayPoint>> &rollOuts, const vector<WayPoint> &totalPaths, const WayPoint &currState, const vector<WayPoint> &contourPoints, const PlanningParams &params, const CAR_BASIC_INFO &carInfo, const VehicleState &vehicleState);
    void CalculateTransitionCosts(vector<OP_TrajectoryCost> &trajectoryCosts, const int &currTrajectoryIndex, const PlanningParams &params);

    void CalculateIntersectionVelocities(const std::vector<WayPoint> &path, const DetectedObject &obj, const WayPoint &currPose, const CAR_BASIC_INFO &carInfo, const double &c_lateral_d, WayPoint &collisionPoint, OP_TrajectoryCost &trajectoryCosts);
    int GetCurrentRollOutIndex(const std::vector<WayPoint> &path, const WayPoint &currState, const PlanningParams &params);
    void InitializeCosts(const vector<vector<WayPoint>> &rollOuts, const PlanningParams &params);
    void InitializeSafetyPolygon(const WayPoint &currState, const CAR_BASIC_INFO &carInfo, const VehicleState &vehicleState, const double &c_lateral_d, const double &c_long_front_d, const double &c_long_back_d);
    void CalculateLateralAndLongitudinalCostsDynamic(const std::vector<DetectedObject> &obj_list, const vector<vector<WayPoint>> &rollOuts, const vector<WayPoint> &totalPaths,
                                                     const WayPoint &currState, const PlanningParams &params, const CAR_BASIC_INFO &carInfo,
                                                     const VehicleState &vehicleState, const double &c_lateral_d, const double &c_long_front_d, const double &c_long_back_d);
};