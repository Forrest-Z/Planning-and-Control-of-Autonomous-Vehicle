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

#include "PlannerH.h"
#include <iostream>

namespace
{
    double diffBetweenTwoAngle(const double &a1, const double &a2)
    {
        double diff = a1 - a2;
        if (diff < 0)
            diff = -diff;
        if (diff > M_PI)
            diff = 2 * M_PI - diff;
        return diff;
    }

    /**
     * @description:将当前角度转换到0～2pi
     * @param {type}
     * @return:
     */
    double cast_from_0_to_2PI_Angle(const double &ang)
    {
        double angle = 0;
        if (ang < -2.0 * M_PI || ang > 2.0 * M_PI)
        {
            angle = fmod(ang, 2.0 * M_PI);
        }
        else
            angle = ang;

        if (angle < 0)
        {
            angle = 2.0 * M_PI + angle;
        }
        return angle;
    }
}

PlannerH::PlannerH()
{
    // m_Params = params;
}

PlannerH::~PlannerH()
{
}
/**
 * @description:生成候选局部规划路径rollouts
 * @param {type}
 * @return:
 */
void PlannerH::GenerateRunoffTrajectory(const std::vector<std::vector<WayPoint>> &referencePaths, const WayPoint &carPos, const bool &bEnableLaneChange, const double &speed, const double &microPlanDistance,
                                        const double &maxSpeed, const double &minSpeed, const double &carTipMargin, const double &rollInMargin,
                                        const double &rollInSpeedFactor, const double &pathDensity, const double &rollOutDensity,
                                        const int &rollOutNumber, const double &SmoothDataWeight, const double &SmoothWeight,
                                        const double &SmoothTolerance, const double &speedProfileFactor, const bool &bHeadingSmooth,
                                        const int &iCurrGlobalPath, const int &iCurrLocalTraj,
                                        std::vector<std::vector<std::vector<WayPoint>>> &rollOutsPaths,
                                        std::vector<WayPoint> &sampledPoints_debug)
{

    if (referencePaths.size() == 0) //参考线
        return;
    if (microPlanDistance <= 0)
        return;
    rollOutsPaths.clear();

    sampledPoints_debug.clear(); // for visualization only

    for (unsigned int i = 0; i < referencePaths.size(); i++)
    {
        std::vector<std::vector<WayPoint>> local_rollOutPaths;
        int s_index = 0, e_index = 0;
        std::vector<double> e_distances;
        if (referencePaths.at(i).size() > 0)
        {

            PlanningHelpers::CalculateRollInTrajectories(carPos, speed, referencePaths.at(i), s_index, e_index, e_distances,
                                                         local_rollOutPaths, microPlanDistance, maxSpeed, carTipMargin, rollInMargin,
                                                         rollInSpeedFactor, pathDensity, rollOutDensity, rollOutNumber,
                                                         SmoothDataWeight, SmoothWeight, SmoothTolerance, bHeadingSmooth, sampledPoints_debug);
            // calculateRollInTrajectories(carPos, speed, referencePaths.at(i), s_index, e_index, e_distances,
            //                             local_rollOutPaths, microPlanDistance, carTipMargin, rollInMargin,
            //                             rollInSpeedFactor, pathDensity, rollOutDensity, rollOutNumber,
            //                             SmoothDataWeight, SmoothWeight, SmoothTolerance, sampledPoints_debug);
        }
        else
        {
            for (int j = 0; j < rollOutNumber + 1; j++)
            {
                local_rollOutPaths.push_back(std::vector<WayPoint>());
            }
        }

        rollOutsPaths.push_back(local_rollOutPaths);
    }
}

/**
 * @description: 由中心轨迹的采样点计算候选rollouts的采样点并平滑
 * @param {type}
 * @return:
 */
void PlannerH::calculateRollInTrajectories(const WayPoint &carPos,
                                           const double &speed,
                                           const std::vector<WayPoint> &originalCenter,
                                           int &start_index,
                                           int &end_index,
                                           std::vector<double> &end_laterals,
                                           std::vector<std::vector<WayPoint>> &rollInPaths,
                                           const double &max_roll_distance,
                                           const double &carTipMargin,
                                           const double &rollInMargin,
                                           const double &rollInSpeedFactor,
                                           const double &pathDensity,
                                           const double &rollOutDensity,
                                           const int &rollOutNumber,
                                           const double &SmoothDataWeight,
                                           const double &SmoothWeight,
                                           const double &SmoothTolerance,
                                           std::vector<WayPoint> &sampledPoints)
{
    WayPoint p;

    int iLimitIndex = (carTipMargin / 0.3) / pathDensity;
    if (iLimitIndex >= originalCenter.size())
        iLimitIndex = originalCenter.size() - 1;

    // Get Closest Index
    RelativeInfo info;
    getRelativeInfo(originalCenter, carPos, info);

    double remaining_distance = 0;
    int close_index = info.iBack;
    for (unsigned int i = close_index; i < originalCenter.size() - 1; i++)
    {
        if (i > 0)
            remaining_distance += distance2points(originalCenter[i].pos, originalCenter[i + 1].pos);
    }

    double initial_roll_in_distance = info.perp_distance; // GetPerpDistanceToTrajectorySimple(originalCenter, carPos, close_index);

    std::vector<WayPoint> RollOutStratPath;

    // calculate the starting index
    double d_limit = 0;
    unsigned int far_index = close_index;

    // calculate end index
    double start_distance = rollInSpeedFactor * speed + rollInMargin;
    if (start_distance > remaining_distance)
        start_distance = remaining_distance;

    d_limit = 0;
    for (unsigned int i = close_index; i < originalCenter.size(); i++)
    {
        if (i > 0)
            d_limit += distance2points(originalCenter[i].pos, originalCenter[i - 1].pos);

        if (d_limit >= start_distance)
        {
            far_index = i;
            break;
        }
    }

    int centralTrajectoryIndex = rollOutNumber / 2;
    std::vector<double> end_distance_list;
    for (int i = 0; i < rollOutNumber + 1; i++)
    {
        double end_roll_in_distance = rollOutDensity * (i - centralTrajectoryIndex);
        end_distance_list.push_back(end_roll_in_distance);
    }

    start_index = close_index;
    end_index = far_index; // end_index是第二个阶段结尾的点坐标
    end_laterals = end_distance_list;

    // calculate the actual calculation starting index
    d_limit = 0;
    unsigned int smoothing_start_index = start_index;
    unsigned int smoothing_end_index = end_index;

    for (unsigned int i = smoothing_start_index; i < originalCenter.size(); i++)
    {
        if (i > 0)
            d_limit += distance2points(originalCenter[i].pos, originalCenter[i - 1].pos);
        if (d_limit > carTipMargin)
            break;

        smoothing_start_index++; // 这个是一个阶段结尾的点下标
    }

    d_limit = 0;
    for (unsigned int i = end_index; i < originalCenter.size(); i++)
    {
        if (i > 0)
            d_limit += distance2points(originalCenter[i].pos, originalCenter[i - 1].pos);
        if (d_limit > carTipMargin)
            break;

        smoothing_end_index++;
    }
    // printf("%s %d %d %d \n", "----------------", int(smoothing_start_index), int(end_index), int(originalCenter.size()));

    int nSteps = end_index - smoothing_start_index;

    std::vector<double> inc_list;
    rollInPaths.clear();
    std::vector<double> inc_list_inc;
    for (int i = 0; i < rollOutNumber + 1; i++)
    {
        double diff = end_laterals.at(i) - initial_roll_in_distance;
        inc_list.push_back(diff / (double)nSteps);
        rollInPaths.push_back(std::vector<WayPoint>());
        inc_list_inc.push_back(0);
    }

    std::vector<std::vector<WayPoint>> execluded_from_smoothing;
    for (unsigned int i = 0; i < rollOutNumber + 1; i++)
        execluded_from_smoothing.push_back(std::vector<WayPoint>());

    // Insert First strait points within the tip of the car range
    for (unsigned int j = start_index; j < smoothing_start_index; j++)
    {
        p = originalCenter.at(j);
        double original_speed = p.v;
        for (unsigned int i = 0; i < rollOutNumber + 1; i++)
        {
            p.pos.x = originalCenter.at(j).pos.x - initial_roll_in_distance * cos(p.pos.a + M_PI_2);
            p.pos.y = originalCenter.at(j).pos.y - initial_roll_in_distance * sin(p.pos.a + M_PI_2);

            if (i != centralTrajectoryIndex)
                p.v = original_speed * LANE_CHANGE_SPEED_FACTOR;
            else
                p.v = original_speed;

            if (j < iLimitIndex)
                execluded_from_smoothing.at(i).push_back(p);
            else
                rollInPaths.at(i).push_back(p);

            sampledPoints.push_back(p);
        }
    }

    for (unsigned int j = smoothing_start_index; j < end_index; j++)
    {
        p = originalCenter.at(j);
        double original_speed = p.v;
        for (unsigned int i = 0; i < rollOutNumber + 1; i++)
        {
            inc_list_inc[i] += inc_list[i];
            double d = inc_list_inc[i];
            p.pos.x = originalCenter.at(j).pos.x - initial_roll_in_distance * cos(p.pos.a + M_PI_2) - d * cos(p.pos.a + M_PI_2);
            p.pos.y = originalCenter.at(j).pos.y - initial_roll_in_distance * sin(p.pos.a + M_PI_2) - d * sin(p.pos.a + M_PI_2);

            if (i != centralTrajectoryIndex)
                p.v = original_speed * LANE_CHANGE_SPEED_FACTOR;
            else
                p.v = original_speed;

            rollInPaths.at(i).push_back(p);

            sampledPoints.push_back(p);
        }
    }
    // Insert last strait points to make better smoothing
    for (unsigned int j = end_index; j < smoothing_end_index; j++)
    {
        p = originalCenter.at(j);
        double original_speed = p.v;
        for (unsigned int i = 0; i < rollOutNumber + 1; i++)
        {
            double d = end_laterals.at(i);
            p.pos.x = originalCenter.at(j).pos.x - d * cos(p.pos.a + M_PI_2);
            p.pos.y = originalCenter.at(j).pos.y - d * sin(p.pos.a + M_PI_2);
            if (i != centralTrajectoryIndex)
                p.v = original_speed * LANE_CHANGE_SPEED_FACTOR;
            else
                p.v = original_speed;
            rollInPaths.at(i).push_back(p);
            sampledPoints.push_back(p);
        }
    }

    for (unsigned int i = 0; i < rollOutNumber + 1; i++)
        rollInPaths.at(i).insert(rollInPaths.at(i).begin(), execluded_from_smoothing.at(i).begin(), execluded_from_smoothing.at(i).end());

    d_limit = 0;
    for (unsigned int j = smoothing_end_index; j < originalCenter.size(); j++)
    {
        if (j > 0)
            d_limit += distance2points(originalCenter.at(j).pos, originalCenter.at(j - 1).pos);

        if (d_limit > max_roll_distance)
            break;
        p = originalCenter.at(j);
        double original_speed = p.v;
        for (unsigned int i = 0; i < rollInPaths.size(); i++)
        {
            double d = end_laterals.at(i);
            p.pos.x = originalCenter.at(j).pos.x - d * cos(p.pos.a + M_PI_2);
            p.pos.y = originalCenter.at(j).pos.y - d * sin(p.pos.a + M_PI_2);

            if (i != centralTrajectoryIndex)
                p.v = original_speed * LANE_CHANGE_SPEED_FACTOR;
            else
                p.v = original_speed;

            rollInPaths.at(i).push_back(p);

            sampledPoints.push_back(p);
        }
    }

    for (unsigned int i = 0; i < rollOutNumber + 1; i++)
    {
        smoothPath(rollInPaths.at(i), SmoothDataWeight, SmoothWeight, SmoothTolerance);
    }
}

/**
 * @description: 平滑生成的曲线
 * @param {type}
 * @return:
 */
void PlannerH::smoothPath(std::vector<WayPoint> &path, double weight_data, double weight_smooth, double tolerance)
{
    if (path.size() <= 2)
        return;

    const std::vector<WayPoint> &path_in = path;
    std::vector<WayPoint> smoothPath_out = path_in;

    double change = tolerance;
    double xtemp, ytemp;
    int nIterations = 0;

    int size = path_in.size();

    while (change >= tolerance)
    {
        change = 0.0;
        for (int i = 1; i < size - 1; i++)
        {
            xtemp = smoothPath_out[i].pos.x;
            ytemp = smoothPath_out[i].pos.y;

            smoothPath_out[i].pos.x += weight_data * (path_in[i].pos.x - smoothPath_out[i].pos.x);
            smoothPath_out[i].pos.y += weight_data * (path_in[i].pos.y - smoothPath_out[i].pos.y);

            smoothPath_out[i].pos.x += weight_smooth * (smoothPath_out[i - 1].pos.x + smoothPath_out[i + 1].pos.x - (2.0 * smoothPath_out[i].pos.x));
            smoothPath_out[i].pos.y += weight_smooth * (smoothPath_out[i - 1].pos.y + smoothPath_out[i + 1].pos.y - (2.0 * smoothPath_out[i].pos.y));

            change += fabs(xtemp - smoothPath_out[i].pos.x);
            change += fabs(ytemp - smoothPath_out[i].pos.y);
        }
        nIterations++;
    }
    path = smoothPath_out;
}

/**
 * @description: 计算某一个轨迹到某一个点的相对位置
 */
bool PlannerH::getRelativeInfo(const std::vector<WayPoint> &trajectory, const WayPoint &p, RelativeInfo &info)
{
    if (trajectory.size() < 2)
        return false;

    WayPoint p0, p1;
    if (trajectory.size() == 2)
    {
        p0 = trajectory[0];
        p1 = WayPoint((p0.pos.x + trajectory[1].pos.x) / 2.0,
                      (p0.pos.y + trajectory[1].pos.y) / 2.0,
                      (p0.pos.z + trajectory[1].pos.z) / 2.0,
                      p0.pos.a);
        info.iBack = 0;
        info.iFront = 1;
    }
    else
    {
        info.iFront = getNextClosePointIndex(trajectory, p);

        if (info.iFront > 0)
            info.iBack = info.iFront - 1;
        else
            info.iBack = 0;

        if (info.iFront == 0)
        {
            p0 = trajectory[info.iFront];
            p1 = trajectory[info.iFront + 1];
        }
        else if (info.iFront > 0 && info.iFront < trajectory.size() - 1)
        {
            p0 = trajectory[info.iFront - 1];
            p1 = trajectory[info.iFront];
        }
        else
        {
            p0 = trajectory[info.iFront - 1];
            p1 = WayPoint((p0.pos.x + trajectory[info.iFront].pos.x) / 2.0,
                          (p0.pos.y + trajectory[info.iFront].pos.y) / 2.0,
                          (p0.pos.z + trajectory[info.iFront].pos.z) / 2.0,
                          p0.pos.a);
        }
    }

    WayPoint prevWP = p0;
    Mat3 rotationMat(-p1.pos.a);
    Mat3 translationMat(-p.pos.x, -p.pos.y);
    Mat3 invRotationMat(p1.pos.a);
    Mat3 invTranslationMat(p.pos.x, p.pos.y);

    p0.pos = translationMat * p0.pos;
    p0.pos = rotationMat * p0.pos;

    p1.pos = translationMat * p1.pos;
    p1.pos = rotationMat * p1.pos;

    double k = (p1.pos.y - p0.pos.y) / (p1.pos.x - p0.pos.x);
    info.perp_distance = p1.pos.y - k * p1.pos.x;

    if (std::isnan(info.perp_distance) || std::isinf(info.perp_distance))
        info.perp_distance = 0;

    info.to_front_distance = fabs(p1.pos.x);

    info.perp_point = p1;
    info.perp_point.pos.x = 0;
    info.perp_point.pos.y = info.perp_distance;

    info.perp_point.pos = invRotationMat * info.perp_point.pos;
    info.perp_point.pos = invTranslationMat * info.perp_point.pos;

    info.from_back_distance = hypot(info.perp_point.pos.y - prevWP.pos.y, info.perp_point.pos.x - prevWP.pos.x);
    info.angle_diff = diffBetweenTwoAngle(p1.pos.a, p.pos.a) * RAD2DEG;

    info.direct_distance = hypot(p1.pos.y - p.pos.y, p1.pos.x - p.pos.x);
    return true;
}

/**
 * @description: 获取轨迹上距离当前位置最近的轨迹点（前方）
 * @param {type}
 * @return:
 */
int PlannerH::getNextClosePointIndex(const std::vector<WayPoint> &trajectory,
                                     const WayPoint &curr_pos,
                                     const int &prevIndex)
{
    if (trajectory.size() < 2 || prevIndex < 0)
        return 0;
    double dis = 0, min_dis = DBL_MAX;
    int min_index = prevIndex;

    for (int i = prevIndex; i < trajectory.size(); i++)
    {
        dis = distance2points_pow(trajectory[i].pos, curr_pos.pos);

        if (dis < min_dis)
        {
            min_index = i;
            min_dis = dis;
        }
    }
    // printf("index %d min_dis %f\n", min_index, min_dis);

    if (min_index < (int)trajectory.size() - 2)
    {
        GpsPoint closest, next;
        closest = trajectory[min_index].pos;
        next = trajectory[min_index + 1].pos;
        GpsPoint v_1(curr_pos.pos.x - closest.x, curr_pos.pos.y - closest.y, 0, 0);
        double length1 = calLength(v_1);
        GpsPoint v_2(next.x - closest.x, next.y - closest.y, 0, 0);
        double length2 = calLength(v_2);
        double angle = cast_from_0_to_2PI_Angle(acos((v_1.x * v_2.x + v_1.y * v_2.y) / (length1 * length2)));
        if (angle <= M_PI_2)
            min_index = min_index + 1;
    }
    return min_index;
}