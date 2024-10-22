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
#include <string>
#include <vector>
#include <sstream>

#define DEG2RAD M_PI / 180.
#define RAD2DEG 180. / M_PI
#define SIGN(x) (x > 0) ? 1 : ((x < 0) ? -1 : 0)
#define MIN(x, y) (x <= y ? x : y)
#define MAX(x, y) (x >= y ? x : y)


enum DIRECTION_TYPE
{
    FORWARD_DIR,
    FORWARD_LEFT_DIR,
    FORWARD_RIGHT_DIR,
    BACKWARD_DIR,
    BACKWARD_LEFT_DIR,
    BACKWARD_RIGHT_DIR,
    STANDSTILL_DIR
};

enum OBSTACLE_TYPE
{
    SIDEWALK,
    TREE,
    CAR,
    TRUCK,
    HOUSE,
    PEDESTRIAN,
    CYCLIST,
    GENERAL_OBSTACLE
};

enum DRIVABLE_TYPE
{
    DIRT,
    TARMAC,
    PARKINGAREA,
    INDOOR,
    GENERAL_AREA
};

enum GLOBAL_STATE_TYPE
{
    G_WAITING_STATE,
    G_PLANING_STATE,
    G_FORWARD_STATE,
    G_BRANCHING_STATE,
    G_FINISH_STATE
};

enum STATE_TYPE
{
    INITIAL_STATE,
    WAITING_STATE,
    FORWARD_STATE,
    STOPPING_STATE,
    EMERGENCY_STATE,
    TRAFFIC_LIGHT_STOP_STATE,
    TRAFFIC_LIGHT_WAIT_STATE,
    STOP_SIGN_STOP_STATE,
    STOP_SIGN_WAIT_STATE,
    FOLLOW_STATE,
    LANE_CHANGE_STATE,
    OBSTACLE_AVOIDANCE_STATE,
    GOAL_STATE,
    FINISH_STATE,
    YIELDING_STATE,
    BRANCH_LEFT_STATE,
    BRANCH_RIGHT_STATE
};

enum LIGHT_INDICATOR
{
    INDICATOR_LEFT,
    INDICATOR_RIGHT,
    INDICATOR_BOTH,
    INDICATOR_NONE
};

enum SHIFT_POS
{
    SHIFT_POS_PP = 0x60,
    SHIFT_POS_RR = 0x40,
    SHIFT_POS_NN = 0x20,
    SHIFT_POS_DD = 0x10,
    SHIFT_POS_BB = 0xA0,
    SHIFT_POS_SS = 0x0f,
    SHIFT_POS_UU = 0xff
};

enum ACTION_TYPE
{
    FORWARD_ACTION,
    BACKWARD_ACTION,
    STOP_ACTION,
    LEFT_TURN_ACTION,
    RIGHT_TURN_ACTION,
    U_TURN_ACTION,
    SWERVE_ACTION,
    OVERTACK_ACTION,
    START_ACTION,
    SLOWDOWN_ACTION,
    CHANGE_DESTINATION,
    WAITING_ACTION,
    DESTINATION_REACHED,
    UNKOWN_ACTION
};

enum BEH_STATE_TYPE
{
    BEH_FORWARD_STATE = 0,
    BEH_STOPPING_STATE = 1,
    BEH_BRANCH_LEFT_STATE = 2,
    BEH_BRANCH_RIGHT_STATE = 3,
    BEH_YIELDING_STATE = 4,
    BEH_ACCELERATING_STATE = 5,
    BEH_SLOWDOWN_STATE = 6
};

enum SEGMENT_TYPE
{
    NORMAL_ROAD_SEG,
    INTERSECTION_ROAD_SEG,
    UTURN_ROAD_SEG,
    EXIT_ROAD_SEG,
    MERGE_ROAD_SEG,
    HIGHWAY_ROAD_SEG
};
enum RoadSegmentType
{
    NORMAL_ROAD,
    INTERSECTION_ROAD,
    UTURN_ROAD,
    EXIT_ROAD,
    MERGE_ROAD,
    HIGHWAY_ROAD
};

enum MARKING_TYPE
{
    UNKNOWN_MARK,
    TEXT_MARK,
    AF_MARK,
    AL_MARK,
    AR_MARK,
    AFL_MARK,
    AFR_MARK,
    ALR_MARK,
    UTURN_MARK,
    NOUTURN_MARK
};

enum TrafficSignTypes
{
    UNKNOWN_SIGN,
    STOP_SIGN,
    MAX_SPEED_SIGN,
    MIN_SPEED_SIGN
};

enum LaneType
{
    NORMAL_LANE,
    MERGE_LANE,
    EXIT_LANE,
    BUS_LANE,
    BUS_STOP_LANE,
    EMERGENCY_LANE
};

enum TrafficLightState
{
    UNKNOWN_LIGHT,
    RED_LIGHT,
    GREEN_LIGHT,
    YELLOW_LIGHT,
    LEFT_GREEN,
    FORWARD_GREEN,
    RIGHT_GREEN,
    FLASH_YELLOW,
    FLASH_RED
};

class Lane;
class OP_TrafficLight;
class RoadSegment;
class WayPoint;

class GpsPoint
{
public:
    double lat, x;
    double lon, y;
    double alt, z;
    double dir, a;

    GpsPoint()
    {
        lat = x = 0;
        lon = y = 0;
        alt = z = 0;
        dir = a = 0;
    }

    GpsPoint(const double &x, const double &y, const double &z, const double &a)
    {
        this->x = x;
        this->y = y;
        this->z = z;
        this->a = a;

        lat = 0;
        lon = 0;
        alt = 0;
        dir = 0;
    }

    std::string ToString()
    {
        std::stringstream str;
        str.precision(12);
        str << "X:" << x << ", Y:" << y << ", Z:" << z << ", A:" << a << std::endl;
        str << "Lon:" << lon << ", Lat:" << lat << ", Alt:" << alt << ", Dir:" << dir << std::endl;
        return str.str();
    }
};

class Boundary // represent wayarea in vector map
{
public:
    int id;
    int roadId;
    std::vector<GpsPoint> points;
    RoadSegment *pSegment;

    Boundary()
    {
        id = 0;
        roadId = 0;
        pSegment = nullptr;
    }
};

class Crossing
{
public:
    int id;
    int roadId;
    std::vector<GpsPoint> points;
    RoadSegment *pSegment;

    Crossing()
    {
        id = 0;
        roadId = 0;
        pSegment = nullptr;
    }
};

class RoadSegment
{
public:
    int id;

    SEGMENT_TYPE roadType;
    Boundary boundary;
    Crossing start_crossing;
    Crossing finish_crossing;
    double avgWidth;
    std::vector<int> fromIds;
    std::vector<int> toIds;
    std::vector<Lane> Lanes;

    std::vector<RoadSegment *> fromLanes;
    std::vector<RoadSegment *> toLanes;

    RoadSegment()
    {
        id = 0;
        avgWidth = 0;
        roadType = NORMAL_ROAD_SEG;
    }
};

class OP_TrafficLight
{
public:
    int id;
    GpsPoint pos;
    TrafficLightState lightState;
    double stoppingDistance;
    std::vector<int> laneIds;
    std::vector<Lane *> pLanes;
    int linkID;

    OP_TrafficLight()
    {
        stoppingDistance = 2;
        id = 0;
        lightState = GREEN_LIGHT;
        linkID = 0;
    }

    bool CheckLane(const int &laneId)
    {
        for (unsigned int i = 0; i < laneIds.size(); i++)
        {
            if (laneId == laneIds.at(i))
                return true;
        }
        return false;
    }
};

class StopLine
{
public:
    int id;
    int laneId;
    int roadId;
    int trafficLightID;
    int stopSignID;
    std::vector<GpsPoint> points;
    Lane *pLane;
    int linkID;

    StopLine()
    {
        id = 0;
        laneId = 0;
        roadId = 0;
        pLane = 0;
        trafficLightID = -1;
        stopSignID = -1;
        linkID = 0;
    }
};

class WaitingLine
{
public:
    int id;
    int laneId;
    int roadId;
    std::vector<GpsPoint> points;
    Lane *pLane;

    WaitingLine()
    {
        id = 0;
        laneId = 0;
        roadId = 0;
        pLane = 0;
    }
};

class Lane
{
public:
    int id;
    int roadId;
    int areaId;
    int fromAreaId;
    int toAreaId;
    std::vector<int> fromIds;
    std::vector<int> toIds;
    int num; // lane number in the road segment from left to right
    double speed;
    double length;
    double dir;
    LaneType type;
    double width;
    std::vector<WayPoint> points;
    std::vector<OP_TrafficLight> trafficlights;
    std::vector<StopLine> stopLines;
    WaitingLine waitingLine;

    std::vector<Lane *> fromLanes;
    std::vector<Lane *> toLanes;
    Lane *pLeftLane;
    Lane *pRightLane;

    RoadSegment *pRoad;

    Lane()
    {
        id = 0;
        num = 0;
        speed = 0;
        length = 0;
        dir = 0;
        type = NORMAL_LANE;
        width = 0;
        pLeftLane = 0;
        pRightLane = 0;
        pRoad = 0;
        roadId = 0;
        areaId = 0;
        fromAreaId = 0;
        toAreaId = 0;
    }
};

class RECTANGLE
{
public:
    GpsPoint bottom_left;
    GpsPoint top_right;
    double width;
    double length;
    bool bObstacle;

    inline bool PointInRect(GpsPoint p)
    {
        return p.x >= bottom_left.x && p.x <= top_right.x && p.y >= bottom_left.y && p.y <= top_right.y;
    }

    inline bool PointInsideRect(GpsPoint p)
    {
        return p.x > bottom_left.x && p.x < top_right.x && p.y > bottom_left.y && p.y < top_right.y;
    }

    inline bool HitTest(GpsPoint p)
    {
        return PointInRect(p) && bObstacle;
    }

    RECTANGLE()
    {
        width = 0;
        length = 0;
        bObstacle = true;
    }

    virtual ~RECTANGLE() {}
};

class Rotation
{
public:
    double x;
    double y;
    double z;
    double w;

    Rotation()
    {
        x = 0;
        y = 0;
        z = 0;
        w = 0;
    }
};

class WayPoint
{
public:
    GpsPoint pos;
    Rotation rot;
    double v;
    double cost;
    double timeCost;
    double totalReward;
    double collisionCost;
    double laneChangeCost;
    int laneId;
    int id;
    int LeftPointId;
    int RightPointId;
    int LeftLnId;
    int RightLnId;
    int stopLineID;
    DIRECTION_TYPE bDir;
    STATE_TYPE state;
    BEH_STATE_TYPE beh_state;
    int iOriginalIndex;

    Lane *pLane;
    WayPoint *pLeft;
    WayPoint *pRight;
    std::vector<int> toIds;
    std::vector<int> fromIds;
    std::vector<WayPoint *> pFronts;
    std::vector<WayPoint *> pBacks;
    std::vector<std::pair<ACTION_TYPE, double>> actionCost;

    int originalMapID;
    int gid;

    WayPoint()
    {
        id = 0;
        v = 0;
        cost = 0;
        laneId = -1;
        pLane = 0;
        pLeft = 0;
        pRight = 0;
        bDir = FORWARD_DIR;
        LeftPointId = 0;
        RightPointId = 0;
        LeftLnId = 0;
        RightLnId = 0;
        timeCost = 0;
        totalReward = 0;
        collisionCost = 0;
        laneChangeCost = 0;
        stopLineID = -1;
        state = INITIAL_STATE;
        beh_state = BEH_STOPPING_STATE;
        iOriginalIndex = 0;

        gid = 0;
        originalMapID = -1;
    }

    WayPoint(const double &x, const double &y, const double &z, const double &a)
    {
        pos.x = x;
        pos.y = y;
        pos.z = z;
        pos.a = a;

        id = 0;
        v = 0;
        cost = 0;
        laneId = -1;
        pLane = 0;
        pLeft = 0;
        pRight = 0;
        bDir = FORWARD_DIR;
        LeftPointId = 0;
        RightPointId = 0;
        LeftLnId = 0;
        RightLnId = 0;
        timeCost = 0;
        totalReward = 0;
        collisionCost = 0;
        laneChangeCost = 0;
        stopLineID = -1;
        iOriginalIndex = 0;
        state = INITIAL_STATE;
        beh_state = BEH_STOPPING_STATE;

        gid = 0;
        originalMapID = -1;
    }
};

class PlanningParams
{
public:
    double maxSpeed;
    double minSpeed;
    double planningDistance;
    double microPlanDistance;
    double carTipMargin;
    double rollInMargin;
    double rollInSpeedFactor;
    double pathDensity;
    double rollOutDensity;
    int rollOutNumber;
    double horizonDistance;
    double minFollowingDistance; // should be bigger than Distance to follow
    double minDistanceToAvoid;   // should be smaller than minFollowingDistance and larger than maxDistanceToAvoid
    double maxDistanceToAvoid;   // should be smaller than minDistanceToAvoid
    double speedProfileFactor;
    double smoothingDataWeight;
    double smoothingSmoothWeight;
    double smoothingToleranceError;

    double stopSignStopTime;

    double additionalBrakingDistance;
    double verticalSafetyDistance;
    double horizontalSafetyDistancel;

    double giveUpDistance;

    int nReliableCount;

    bool enableLaneChange;
    bool enableSwerving;
    bool enableFollowing;
    bool enableHeadingSmoothing;
    bool enableTrafficLightBehavior;
    bool enableStopSignBehavior;

    bool enabTrajectoryVelocities;
    double minIndicationDistance;

    PlanningParams()
    {
        maxSpeed = 3;
        minSpeed = 0;
        planningDistance = 10000;
        microPlanDistance = 30;
        carTipMargin = 4.0;
        rollInMargin = 12.0;
        rollInSpeedFactor = 0.25;
        pathDensity = 0.25;
        rollOutDensity = 0.5;
        rollOutNumber = 4;
        horizonDistance = 120;
        minFollowingDistance = 35;
        minDistanceToAvoid = 15;
        maxDistanceToAvoid = 5;
        speedProfileFactor = 1.0;
        smoothingDataWeight = 0.47;
        smoothingSmoothWeight = 0.2;
        smoothingToleranceError = 0.05;

        stopSignStopTime = 2.0;

        additionalBrakingDistance = 10.0;
        verticalSafetyDistance = 0.0;
        horizontalSafetyDistancel = 0.0;

        giveUpDistance = -4;
        nReliableCount = 2;

        enableHeadingSmoothing = false;
        enableSwerving = false;
        enableFollowing = false;
        enableTrafficLightBehavior = false;
        enableLaneChange = false;
        enableStopSignBehavior = false;
        enabTrajectoryVelocities = false;
        minIndicationDistance = 15;
    }
};

class Curb
{
public:
    int id;
    int laneId;
    int roadId;
    std::vector<GpsPoint> points;
    Lane *pLane;

    Curb()
    {
        id = 0;
        laneId = 0;
        roadId = 0;
        pLane = 0;
    }
};

class Marking
{
public:
    int id;
    int laneId;
    int roadId;
    MARKING_TYPE mark_type;
    GpsPoint center;
    std::vector<GpsPoint> points;
    Lane *pLane;

    Marking()
    {
        id = 0;
        laneId = 0;
        roadId = 0;
        mark_type = UNKNOWN_MARK;
        pLane = nullptr;
    }
};

class OP_TrafficSign
{
public:
    int id;
    int laneId;
    int roadId;

    GpsPoint pos;
    TrafficSignTypes signType;
    double value;
    double fromValue;
    double toValue;
    std::string strValue;
    timespec timeValue;
    timespec fromTimeValue;
    timespec toTimeValue;

    Lane *pLane;

    OP_TrafficSign()
    {
        id = 0;
        laneId = 0;
        roadId = 0;
        signType = UNKNOWN_SIGN;
        value = 0;
        fromValue = 0;
        toValue = 0;
        //    timeValue  = 0;
        //    fromTimeValue = 0;
        //    toTimeValue  = 0;
        pLane = 0;
    }
};

class RoadNetwork
{
public:
    std::vector<RoadSegment> roadSegments;
    std::vector<OP_TrafficLight> trafficLights;
    std::vector<StopLine> stopLines;
    std::vector<Curb> curbs;
    std::vector<Boundary> boundaries;
    std::vector<Crossing> crossings;
    std::vector<Marking> markings;
    std::vector<OP_TrafficSign> signs;
};

class RelativeInfo
{
public:
    double perp_distance;
    double to_front_distance; // negative
    double from_back_distance;
    int iFront;
    int iBack;
    int iGlobalPath;
    WayPoint perp_point;
    double angle_diff; // degrees
    double direct_distance;
    bool bBefore;
    bool bAfter;
    double after_angle;

    RelativeInfo()
    {
        after_angle = 0;
        bBefore = false;
        bAfter = false;
        perp_distance = 0;
        to_front_distance = 0;
        from_back_distance = 0;
        iFront = 0;
        iBack = 0;
        iGlobalPath = 0;
        angle_diff = 0;
    }
};

class DetectedObject
{
public:
    int id;
    std::string label;
    OBSTACLE_TYPE t;
    WayPoint center;
    WayPoint predicted_center;
    WayPoint noisy_center;
    STATE_TYPE predicted_behavior;
    std::vector<WayPoint> centers_list;
    std::vector<GpsPoint> contour;
    std::vector<std::vector<WayPoint>> predTrajectories;
    std::vector<WayPoint *> pClosestWaypoints;
    double w;
    double l;
    double h;
    double distance_to_center;

    double actual_speed;
    double actual_yaw;

    bool bDirection;
    bool bVelocity;
    int acceleration;

    int acceleration_desc;
    double acceleration_raw;

    LIGHT_INDICATOR indicator_state;

    int originalID;
    BEH_STATE_TYPE behavior_state;

    DetectedObject()
    {
        bDirection = false;
        bVelocity = false;
        acceleration = 0;
        acceleration_desc = 0;
        acceleration_raw = 0.0;
        id = 0;
        w = 0;
        l = 0;
        h = 0;
        t = GENERAL_OBSTACLE;
        distance_to_center = 0;
        predicted_behavior = INITIAL_STATE;
        actual_speed = 0;
        actual_yaw = 0;

        acceleration_desc = 0;
        acceleration_raw = 0.0;
        indicator_state = INDICATOR_NONE;

        originalID = -1;
        behavior_state = BEH_STOPPING_STATE;
    }
};

class ObjTimeStamp
{
public:
    timespec tStamp;

    ObjTimeStamp()
    {
        tStamp.tv_nsec = 0;
        tStamp.tv_sec = 0;
    }
};

class VehicleState : public ObjTimeStamp
{
public:
    double speed;
    double steer;
    SHIFT_POS shift;

    VehicleState()
    {
        speed = 0;
        steer = 0;
        shift = SHIFT_POS_NN;
    }
};

class CAR_BASIC_INFO
{
public:
    double turning_radius;
    double wheel_base;
    double max_speed_forward;
    double min_speed_forward;
    double max_speed_backword;
    double max_steer_value;
    double min_steer_value;
    double max_brake_value;
    double min_brake_value;
    double max_steer_angle;
    double min_steer_angle;
    double length;
    double width;
    double max_acceleration;
    double max_deceleration;

    CAR_BASIC_INFO()
    {
        turning_radius = 5.2;
        wheel_base = 2.7;
        max_speed_forward = 3.0;
        min_speed_forward = 0.0;
        max_speed_backword = 1.0;
        max_steer_value = 660;
        min_steer_value = -660;
        max_brake_value = 0;
        min_brake_value = 0;
        max_steer_angle = 0.42;
        min_steer_angle = 0.42;
        length = 4.3;
        width = 1.82;
        max_acceleration = 1.5;  // m/s2
        max_deceleration = -1.5; // 1/3 G
    }

    double CalcMaxSteeringAngle()
    {
        return max_steer_angle; // asin(wheel_base/turning_radius);
    }

    double BoundSpeed(double s)
    {
        if (s > 0 && s > max_speed_forward)
            return max_speed_forward;
        if (s < 0 && s < max_speed_backword)
            return max_speed_backword;
        return s;
    }

    double BoundSteerAngle(double a)
    {
        if (a > max_steer_angle)
            return max_steer_angle;
        if (a < min_steer_angle)
            return min_steer_angle;

        return a;
    }

    double BoundSteerValue(double v)
    {
        if (v >= max_steer_value)
            return max_steer_value;
        if (v <= min_steer_value)
            return min_steer_value;

        return v;
    }
};

class OP_TrajectoryCost
{
public:
    int index;
    int relative_index;
    double closest_obj_velocity;
    double distance_from_center;
    double priority_cost;    // 0 to 1
    double transition_cost;  // 0 to 1
    double closest_obj_cost; // 0 to 1
    double cost;
    double closest_obj_distance;

    int lane_index;
    double lane_change_cost;
    double lateral_cost;
    double longitudinal_cost;
    bool bBlocked;
    std::vector<std::pair<int, double>> lateral_costs;

    OP_TrajectoryCost()
    {
        lane_index = -1;
        index = -1;
        relative_index = -100;
        closest_obj_velocity = 0;
        priority_cost = 0;
        transition_cost = 0;
        closest_obj_cost = 0;
        distance_from_center = 0;
        cost = 0;
        closest_obj_distance = -1;
        lane_change_cost = 0;
        lateral_cost = 0;
        longitudinal_cost = 0;
        bBlocked = false;
    }

    std::string ToString()
    {
        std::ostringstream str;
        str.precision(4);
        str << "LI   : " << lane_index;
        str << ", In : " << relative_index;
        str << ", Co : " << cost;
        str << ", Pr : " << priority_cost;
        str << ", Tr : " << transition_cost;
        str << ", La : " << lateral_cost;
        str << ", Lo : " << longitudinal_cost;
        str << ", Ln : " << lane_change_cost;
        str << ", Bl : " << bBlocked;
        str << "\n";
        for (unsigned int i = 0; i < lateral_costs.size(); i++)
        {
            str << " - (" << lateral_costs.at(i).first << ", " << lateral_costs.at(i).second << ")";
        }

        return str.str();
    }
};

class PolygonShape
{
public:
    std::vector<GpsPoint> points;

    inline int PointInsidePolygon(const PolygonShape &polygon, const GpsPoint &p)
    {
        int counter = 0;
        int i;
        double xinters;
        GpsPoint p1, p2;
        int N = polygon.points.size();
        if (N <= 0)
            return -1;

        p1 = polygon.points.at(0);
        for (i = 1; i <= N; i++)
        {
            p2 = polygon.points.at(i % N);

            if (p.y > MIN(p1.y, p2.y))
            {
                if (p.y <= MAX(p1.y, p2.y))
                {
                    if (p.x <= MAX(p1.x, p2.x))
                    {
                        if (p1.y != p2.y)
                        {
                            xinters = (p.y - p1.y) * (p2.x - p1.x) / (p2.y - p1.y) + p1.x;
                            if (p1.x == p2.x || p.x <= xinters)
                                counter++;
                        }
                    }
                }
            }
            p1 = p2;
        }

        if (counter % 2 == 0)
            return 0;
        else
            return 1;
    }
};
 