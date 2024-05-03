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
#include "PlanningHelpers.h"
#include <string>
#include <float.h>

void PlanningHelpers::CalculateRollInTrajectories(const WayPoint &carPos, const double &speed, const std::vector<WayPoint> &originalCenter, int &start_index,
                                                  int &end_index, std::vector<double> &end_laterals,
                                                  std::vector<std::vector<WayPoint>> &rollInPaths, const double &max_roll_distance,
                                                  const double &maxSpeed, const double &carTipMargin, const double &rollInMargin,
                                                  const double &rollInSpeedFactor, const double &pathDensity, const double &rollOutDensity,
                                                  const int &rollOutNumber, const double &SmoothDataWeight, const double &SmoothWeight,
                                                  const double &SmoothTolerance, const bool &bHeadingSmooth,
                                                  std::vector<WayPoint> &sampledPoints)
{
    WayPoint p;
    double dummyd = 0;

    int iLimitIndex = (carTipMargin / 0.3) / pathDensity;
    if (iLimitIndex >= originalCenter.size())
        iLimitIndex = originalCenter.size() - 1;

    // Get Closest Index
    RelativeInfo info;
    GetRelativeInfo(originalCenter, carPos, info);
    double remaining_distance = 0;
    int close_index = info.iBack;
    for (unsigned int i = close_index; i < originalCenter.size() - 1; i++)
    {
        if (i > 0)
            remaining_distance += distance2points(originalCenter[i].pos, originalCenter[i + 1].pos);
    }

    double initial_roll_in_distance = info.perp_distance; // GetPerpDistanceToTrajectorySimple(originalCenter, carPos, close_index);

    std::vector<WayPoint> RollOutStratPath;

    ///***   Smoothing From Car Heading Section ***///
    //  if(bHeadingSmooth)
    //  {
    //    unsigned int num_of_strait_points = carTipMargin / pathDensity;
    //    int closest_for_each_iteration = 0;
    //    WayPoint np = GetPerpendicularOnTrajectory_obsolete(originalCenter, carPos, dummyd, closest_for_each_iteration);
    //    np.pos = carPos.pos;
    //
    //    RollOutStratPath.push_back(np);
    //    for(unsigned int i = 0; i < num_of_strait_points; i++)
    //    {
    //      p = RollOutStratPath.at(i);
    //      p.pos.x = p.pos.x +  pathDensity*cos(p.pos.a);
    //      p.pos.y = p.pos.y +  pathDensity*sin(p.pos.a);
    //      np = GetPerpendicularOnTrajectory_obsolete(originalCenter, p, dummyd, closest_for_each_iteration);
    //      np.pos = p.pos;
    //      RollOutStratPath.push_back(np);
    //    }
    //
    //    initial_roll_in_distance = GetPerpDistanceToTrajectorySimple_obsolete(originalCenter, RollOutStratPath.at(RollOutStratPath.size()-1), close_index);
    //  }
    ///***   -------------------------------- ***///

    // printf("\n Lateral Distance: %f" , initial_roll_in_distance);

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
    // rollOut 平行横向采样点到最大规划距离
    int centralTrajectoryIndex = rollOutNumber / 2;
    // std::cout << "centralTrajectoryIndex:" << centralTrajectoryIndex << "\n";

    std::vector<double> end_distance_list;
    for (int i = 0; i < rollOutNumber + 1; i++)
    {
        double end_roll_in_distance = rollOutDensity * (i - centralTrajectoryIndex);
        end_distance_list.push_back(end_roll_in_distance);
        // std::cout << "end_roll_in_distance:" << end_roll_in_distance << "\n";
    }

    start_index = close_index;
    end_index = far_index;
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

        smoothing_start_index++;
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

    ///***   Smoothing From Car Heading Section ***///
    //  if(bHeadingSmooth)
    //  {
    //    for(unsigned int i=0; i< rollOutNumber+1 ; i++)
    //    {
    //      unsigned int cut_index = GetClosestNextPointIndex(rollInPaths.at(i), RollOutStratPath.at(RollOutStratPath.size()-1));
    //      rollInPaths.at(i).erase(rollInPaths.at(i).begin(), rollInPaths.at(i).begin()+cut_index);
    //      rollInPaths.at(i).insert(rollInPaths.at(i).begin(), RollOutStratPath.begin(), RollOutStratPath.end());
    //    }
    //  }
    ///***   -------------------------------- ***///

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
        SmoothPath(rollInPaths.at(i), SmoothDataWeight, SmoothWeight, SmoothTolerance);
    }

    //  for(unsigned int i=0; i< rollInPaths.size(); i++)
    //    CalcAngleAndCost(rollInPaths.at(i));
}

void PlanningHelpers::SmoothPath(std::vector<WayPoint> &path, double weight_data,
                                 double weight_smooth, double tolerance)
{

    if (path.size() <= 2)
    {
        // cout << "Can't Smooth Path, Path_in Size=" << path.size() << endl;
        return;
    }

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
            //      if (smoothPath_out[i].pos.a != smoothPath_out[i - 1].pos.a)
            //        continue;

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
double PlanningHelpers::CalcAngleAndCost(std::vector<WayPoint> &path, const double &lastCost, const bool &bSmooth)
{
    if (path.size() < 2)
        return 0;
    if (path.size() == 2)
    {
        path[0].pos.a = FixNegativeAngle(atan2(path[1].pos.y - path[0].pos.y, path[1].pos.x - path[0].pos.x));
        path[0].cost = lastCost;
        path[1].pos.a = path[0].pos.a;
        path[1].cost = path[0].cost + distance2points(path[0].pos, path[1].pos);
        return path[1].cost;
    }

    path[0].pos.a = FixNegativeAngle(atan2(path[1].pos.y - path[0].pos.y, path[1].pos.x - path[0].pos.x));
    path[0].cost = lastCost;

    for (int j = 1; j < path.size() - 1; j++)
    {
        path[j].pos.a = FixNegativeAngle(atan2(path[j + 1].pos.y - path[j].pos.y, path[j + 1].pos.x - path[j].pos.x));
        path[j].cost = path[j - 1].cost + distance2points(path[j - 1].pos, path[j].pos);
    }

    int j = (int)path.size() - 1;

    path[j].pos.a = path[j - 1].pos.a;
    path[j].cost = path[j - 1].cost + distance2points(path[j - 1].pos, path[j].pos);

    for (int j = 0; j < path.size() - 1; j++)
    {
        if (path.at(j).pos.x == path.at(j + 1).pos.x && path.at(j).pos.y == path.at(j + 1).pos.y)
            path.at(j).pos.a = path.at(j + 1).pos.a;
    }

    return path[j].cost;
}

double PlanningHelpers::FixNegativeAngle(const double &a)
{
    double angle = 0;
    if (a < -2.0 * M_PI || a >= 2.0 * M_PI)
    {
        angle = fmod(a, 2.0 * M_PI);
    }
    else
    {
        angle = a;
    }

    if (angle < 0)
    {
        angle = 2.0 * M_PI + angle;
    }

    return angle;
}

double PlanningHelpers::AngleBetweenTwoAnglesPositive(const double &a1, const double &a2)
{
    double diff = a1 - a2;
    if (diff < 0)
        diff = a2 - a1;

    if (diff > M_PI)
        diff = 2.0 * M_PI - diff;

    return diff;
}

bool PlanningHelpers::GetRelativeInfo(const std::vector<WayPoint> &trajectory, const WayPoint &p, RelativeInfo &info, const int &prevIndex)
{
    if (trajectory.size() < 2)
        return false;

    WayPoint p0, p1;
    if (trajectory.size() == 2)
    {
        p0 = trajectory.at(0);
        p1 = WayPoint((trajectory.at(0).pos.x + trajectory.at(1).pos.x) / 2.0,
                      (trajectory.at(0).pos.y + trajectory.at(1).pos.y) / 2.0,
                      (trajectory.at(0).pos.z + trajectory.at(1).pos.z) / 2.0, trajectory.at(0).pos.a);
        info.iFront = 1;
        info.iBack = 0;
    }
    else
    {
        info.iFront = GetClosestNextPointIndexFast(trajectory, p, prevIndex);
        //    WayPoint p2 = p;
        //    int old_index = GetClosestNextPointIndex(trajectory, p2, prevIndex);
        //    if(old_index != info.iFront)
        //      cout << " Alert Alert !!!! fast: " << info.iFront << ", slow: " << old_index  << endl;

        if (info.iFront > 0)
            info.iBack = info.iFront - 1;
        else
            info.iBack = 0;

        if (info.iFront == 0)
        {
            p0 = trajectory.at(info.iFront);
            p1 = trajectory.at(info.iFront + 1);
        }
        else if (info.iFront > 0 && info.iFront < trajectory.size() - 1)
        {
            p0 = trajectory.at(info.iFront - 1);
            p1 = trajectory.at(info.iFront);
        }
        else
        {
            p0 = trajectory.at(info.iFront - 1);
            p1 = WayPoint((p0.pos.x + trajectory.at(info.iFront).pos.x) / 2.0, (p0.pos.y + trajectory.at(info.iFront).pos.y) / 2.0, (p0.pos.z + trajectory.at(info.iFront).pos.z) / 2.0, p0.pos.a);
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

    double m = (p1.pos.y - p0.pos.y) / (p1.pos.x - p0.pos.x);
    info.perp_distance = p1.pos.y - m * p1.pos.x; // solve for x = 0

    if (std::isnan(info.perp_distance) || std::isinf(info.perp_distance))
        info.perp_distance = 0;

    info.to_front_distance = fabs(p1.pos.x); // distance on the x axes

    info.perp_point = p1;
    info.perp_point.pos.x = 0;                  // on the same y axis of the car
    info.perp_point.pos.y = info.perp_distance; // perp distance between the car and the trajectory

    info.perp_point.pos = invRotationMat * info.perp_point.pos;
    info.perp_point.pos = invTranslationMat * info.perp_point.pos;

    info.from_back_distance = hypot(info.perp_point.pos.y - prevWP.pos.y, info.perp_point.pos.x - prevWP.pos.x);

    info.angle_diff = AngleBetweenTwoAnglesPositive(p1.pos.a, p.pos.a) * RAD2DEG;

    return true;
}

int PlanningHelpers::GetClosestNextPointIndexFast(const std::vector<WayPoint> &trajectory, const WayPoint &p, const int &prevIndex)
{
    int size = (int)trajectory.size();

    if (size < 2 || prevIndex < 0)
        return 0;

    double d = 0, minD = DBL_MAX;
    int min_index = prevIndex;
    int iStart = prevIndex;
    int iEnd = size;
    double resolution = hypot(trajectory[1].pos.y - trajectory[0].pos.y, trajectory[1].pos.x - trajectory[0].pos.x);

    // divide every 5 meters
    int skip_factor = 5;
    if (resolution > skip_factor)
        resolution = skip_factor;

    int skip = 1;
    if (resolution > 0)
        skip = skip_factor / resolution;

    for (int i = 0; i < size; i += skip)
    {
        if (i + skip / 2 < size)
            d = (distance2pointsSqr(trajectory[i].pos, p.pos) + distance2pointsSqr(trajectory[i + skip / 2].pos, p.pos)) / 2.0;
        else
            d = distance2pointsSqr(trajectory[i].pos, p.pos);
        if (d < minD)
        {
            iStart = i - skip;
            iEnd = i + skip;
            minD = d;
            min_index = i;
        }
    }

    if ((size - skip / 2 - 1) > 0)
        d = (distance2pointsSqr(trajectory[size - 1].pos, p.pos) + distance2pointsSqr(trajectory[size - skip / 2 - 1].pos, p.pos)) / 2.0;
    else
        d = distance2pointsSqr(trajectory[size - 1].pos, p.pos);

    if (d < minD)
    {
        iStart = size - skip;
        iEnd = size + skip;
        minD = d;
        min_index = size - 1;
    }

    if (iStart < 0)
        iStart = 0;
    if (iEnd >= size)
        iEnd = size - 1;

    for (int i = iStart; i < iEnd; i++)
    {
        d = distance2pointsSqr(trajectory[i].pos, p.pos);
        if (d < minD)
        {
            min_index = i;
            minD = d;
        }
    }

    if (min_index < size - 1)
    {
        GpsPoint curr, next;
        curr = trajectory[min_index].pos;
        next = trajectory[min_index + 1].pos;
        GpsPoint v_1(p.pos.x - curr.x, p.pos.y - curr.y, 0, 0);
        double norm1 = pointNorm(v_1);
        GpsPoint v_2(next.x - curr.x, next.y - curr.y, 0, 0);
        double norm2 = pointNorm(v_2);
        double dot_pro = v_1.x * v_2.x + v_1.y * v_2.y;
        double a = FixNegativeAngle(acos(dot_pro / (norm1 * norm2)));
        if (a <= M_PI_2)
            min_index = min_index + 1;
    }

    // m_TestingClosestPoint.push_back(make_pair(trajectory.at(min_index).pos, p.pos));

    return min_index;
}

void PlanningHelpers::PredictConstantTimeCostForTrajectory(std::vector<WayPoint> &path, const WayPoint &currPose, const double &minVelocity, const double &minDist)
{
    if (path.size() == 0)
        return;

    for (unsigned int i = 0; i < path.size(); i++)
        path.at(i).timeCost = -1;

    if (currPose.v == 0 || currPose.v < minVelocity)
        return;

    RelativeInfo info;
    PlanningHelpers::GetRelativeInfo(path, currPose, info);

    double total_distance = 0;
    double accum_time = 0;

    path.at(info.iFront).timeCost = 0;
    if (info.iFront == 0)
        info.iFront++;

    for (unsigned int i = info.iFront; i < path.size(); i++)
    {
        total_distance += hypot(path.at(i).pos.x - path.at(i - 1).pos.x, path.at(i).pos.y - path.at(i - 1).pos.y);
        accum_time = total_distance / currPose.v;
        path.at(i).timeCost = accum_time;
    }
}

double PlanningHelpers::GetExactDistanceOnTrajectory(const std::vector<WayPoint> &trajectory, const RelativeInfo &p1, const RelativeInfo &p2)
{
    if (trajectory.size() == 0)
        return 0;

    if (p2.iFront == p1.iFront && p2.iBack == p1.iBack)
    {
        return p2.to_front_distance - p1.to_front_distance;
    }
    else if (p2.iBack >= p1.iFront)
    {
        double d_on_path = p1.to_front_distance + p2.from_back_distance;
        for (int i = p1.iFront; i < p2.iBack; i++)
            d_on_path += hypot(trajectory.at(i + 1).pos.y - trajectory.at(i).pos.y, trajectory.at(i + 1).pos.x - trajectory.at(i).pos.x);

        return d_on_path;
    }
    else if (p2.iFront <= p1.iBack)
    {
        double d_on_path = p1.from_back_distance + p2.to_front_distance;
        for (int i = p2.iFront; i < p1.iBack; i++)
            d_on_path += hypot(trajectory.at(i + 1).pos.y - trajectory.at(i).pos.y, trajectory.at(i + 1).pos.x - trajectory.at(i).pos.x);

        return -d_on_path;
    }
    else
    {
        return 0;
    }
}