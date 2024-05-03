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
#include "RoadNetwork.h"
#include "tinyxml.h"
#include <math.h>

#define distance2points(from, to) sqrt(pow(to.x - from.x, 2) + pow(to.y - from.y, 2))
#define distance2pointsSqr(from, to) pow(to.x - from.x, 2) + pow(to.y - from.y, 2)
#define pointNorm(v) sqrt(v.x *v.x + v.y * v.y)
#define angle2points(from, to) atan2(to.y - from.y, to.x - from.x)
#define LANE_CHANGE_SPEED_FACTOR 0.5
#define LANE_CHANGE_COST 3.0             // meters
#define BACKUP_STRAIGHT_PLAN_DISTANCE 75 // meters
#define LANE_CHANGE_MIN_DISTANCE 5

class PlanningHelpers
{

public:
    static std::vector<std::pair<GpsPoint, GpsPoint>> m_TestingClosestPoint;

public:
    PlanningHelpers();
    virtual ~PlanningHelpers();

    static bool GetRelativeInfo(const std::vector<WayPoint> &trajectory, const WayPoint &p, RelativeInfo &info, const int &prevIndex = 0);

    static void SmoothPath(std::vector<WayPoint> &path, double weight_data = 0.25, double weight_smooth = 0.25, double tolerance = 0.01);

    static double CalcAngleAndCost(std::vector<WayPoint> &path, const double &lastCost = 0, const bool &bSmooth = true);

    static int GetClosestNextPointIndexFast(const std::vector<WayPoint> &trajectory, const WayPoint &p, const int &prevIndex = 0);

    static double GetExactDistanceOnTrajectory(const std::vector<WayPoint> &trajectory, const RelativeInfo &p1, const RelativeInfo &p2);

    static void PredictConstantTimeCostForTrajectory(std::vector<WayPoint> &path, const WayPoint &currPose, const double &minVelocity, const double &minDist);

    static void CalculateRollInTrajectories(const WayPoint &carPos, const double &speed, const std::vector<WayPoint> &originalCenter, int &start_index,
                                            int &end_index, std::vector<double> &end_laterals,
                                            std::vector<std::vector<WayPoint>> &rollInPaths, const double &max_roll_distance,
                                            const double &maxSpeed, const double &carTipMargin, const double &rollInMargin,
                                            const double &rollInSpeedFactor, const double &pathDensity, const double &rollOutDensity,
                                            const int &rollOutNumber, const double &SmoothDataWeight, const double &SmoothWeight,
                                            const double &SmoothTolerance, const bool &bHeadingSmooth,
                                            std::vector<WayPoint> &sampledPoints);
    static double FixNegativeAngle(const double &a);
    static double AngleBetweenTwoAnglesPositive(const double &a1, const double &a2);
};

class Mat3
{
    double m[3][3];

public:
    Mat3()
    {
        // initialize Identity by default
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                m[i][j] = 0;

        m[0][0] = m[1][1] = m[2][2] = 1;
    }

    Mat3(double transX, double transY, bool mirrorX, bool mirrorY)
    {
        m[0][0] = (mirrorX == true) ? -1 : 1;
        m[0][1] = 0;
        m[0][2] = transX;
        m[1][0] = 0;
        m[1][1] = (mirrorY == true) ? -1 : 1;
        m[1][2] = transY;
        m[2][0] = 0;
        m[2][1] = 0;
        m[2][2] = 1;
    }

    Mat3(double transX, double transY)
    {
        m[0][0] = 1;
        m[0][1] = 0;
        m[0][2] = transX;
        m[1][0] = 0;
        m[1][1] = 1;
        m[1][2] = transY;
        m[2][0] = 0;
        m[2][1] = 0;
        m[2][2] = 1;
    }

    Mat3(double rotation_angle)
    {
        double c = cos(rotation_angle);
        double s = sin(rotation_angle);
        m[0][0] = c;
        m[0][1] = -s;
        m[0][2] = 0;
        m[1][0] = s;
        m[1][1] = c;
        m[1][2] = 0;
        m[2][0] = 0;
        m[2][1] = 0;
        m[2][2] = 1;
    }

    Mat3(GpsPoint rotationCenter)
    {
        double c = cos(rotationCenter.a);
        double s = sin(rotationCenter.a);
        double u = rotationCenter.x;
        double v = rotationCenter.y;
        m[0][0] = c;
        m[0][1] = -s;
        m[0][2] = -u * c + v * s + u;
        m[1][0] = s;
        m[1][1] = c;
        m[1][2] = -u * s - v * c + v;
        m[2][0] = 0;
        m[2][1] = 0;
        m[2][2] = 1;
    }

    GpsPoint operator*(GpsPoint v)
    {
        GpsPoint _v = v;
        v.x = m[0][0] * _v.x + m[0][1] * _v.y + m[0][2] * 1;
        v.y = m[1][0] * _v.x + m[1][1] * _v.y + m[1][2] * 1;
        return v;
    }
};