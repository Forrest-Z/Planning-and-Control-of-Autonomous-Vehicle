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
#include "PlanningHelpers.h"
#include "RoadNetwork.h"
#include <float.h>
#include <math.h>
#include <string>
 
#define distance2points(from, to) sqrt(pow(to.x - from.x, 2) + pow(to.y - from.y, 2))
#define distance2points_pow(from, to) pow(to.x - from.x, 2) + pow(to.y - from.y, 2)
#define calLength(v) sqrt(v.x *v.x + v.y * v.y)
#define DEG2RAD M_PI / 180.
#define RAD2DEG 180. / M_PI

#define LANE_CHANGE_SPEED_FACTOR 0.5
#define START_POINT_MAX_DISTANCE 8           // meters
#define GOAL_POINT_MAX_DISTANCE 8            // meters
#define LANE_CHANGE_SMOOTH_FACTOR_DISTANCE 8 // meters

enum PLANDIRECTION
{
    MOVE_FORWARD_ONLY,
    MOVE_BACKWARD_ONLY,
    MOVE_FREE
};
enum HeuristicConstrains
{
    EUCLIDEAN,
    NEIGBORHOOD,
    DIRECTION
};

class PlannerH
{
public:
    PlannerH();
    virtual ~PlannerH();

    void GenerateRunoffTrajectory(const std::vector<std::vector<WayPoint>> &referencePaths, const WayPoint &carPos, const bool &bEnableLaneChange, const double &speed, const double &microPlanDistance,
                                  const double &maxSpeed, const double &minSpeed, const double &carTipMargin, const double &rollInMargin,
                                  const double &rollInSpeedFactor, const double &pathDensity, const double &rollOutDensity,
                                  const int &rollOutNumber, const double &SmoothDataWeight, const double &SmoothWeight,
                                  const double &SmoothTolerance, const double &speedProfileFactor, const bool &bHeadingSmooth,
                                  const int &iCurrGlobalPath, const int &iCurrLocalTraj,
                                  std::vector<std::vector<std::vector<WayPoint>>> &rollOutsPaths,
                                  std::vector<WayPoint> &sampledPoints);
void smoothPath(std::vector<WayPoint>& path, double weight_data, double weight_smooth, double tolerance);

bool getRelativeInfo(const std::vector< WayPoint>& trajectory,const  WayPoint& p, RelativeInfo& info);

void  calculateRollInTrajectories(const  WayPoint& carPos,
    const double& speed,
    const std::vector< WayPoint>& originalCenter,
    int& start_index,
    int& end_index,
    std::vector<double>& end_laterals,
    std::vector<std::vector< WayPoint>>& rollInPaths,
    const double& max_roll_distance,
    const double& carTipMargin,
    const double& rollInMargin,
    const double& rollInSpeedFactor,
    const double& pathDensity,
    const double& rollOutDensity,
    const int& rollOutNumber,
    const double& SmoothDataWeight,
    const double& SmoothWeight,
    const double& SmoothTolerance,
    std::vector< WayPoint>& sampledPoints);

   int getNextClosePointIndex(const std::vector<WayPoint> &trajectory,const WayPoint &curr_pos,const int &prevIndex = 0);

    double PlanUsingDP(const WayPoint &carPos, const WayPoint &goalPos,
                       const double &maxPlanningDistance, const bool bEnableLaneChange, const std::vector<int> &globalPath,
                       RoadNetwork &map, std::vector<std::vector<WayPoint>> &paths, std::vector<WayPoint *> *all_cell_to_delete = 0);

    double PlanUsingDPRandom(const WayPoint &start,
                             const double &maxPlanningDistance,
                             RoadNetwork &map,
                             std::vector<std::vector<WayPoint>> &paths);

    double PredictPlanUsingDP(Lane *lane, const WayPoint &carPos, const double &maxPlanningDistance,
                              std::vector<std::vector<WayPoint>> &paths);

    double PredictPlanUsingDP(const WayPoint &startPose, WayPoint *closestWP, const double &maxPlanningDistance, std::vector<std::vector<WayPoint>> &paths, const bool &bFindBranches = true);

    double PredictTrajectoriesUsingDP(const WayPoint &startPose, std::vector<WayPoint *> closestWPs, const double &maxPlanningDistance, std::vector<std::vector<WayPoint>> &paths, const bool &bFindBranches = true, const bool bDirectionBased = false, const bool pathDensity = 1.0);

    void DeleteWaypoints(std::vector<WayPoint *> &wps);
};