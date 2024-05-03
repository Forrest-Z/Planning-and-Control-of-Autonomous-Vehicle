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
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/tf.h>
#include "reference_point.h"
#include "trajectoryPoint.h"
#include "PlannerH.h"
#include "TrajectoryDynamicCosts.h"
#include "Obstacle.h"


class OP_Lane
{
public:
    OP_Lane()
    {
        waypoints = {};
        transition_cost = 0;
        center_cost = 0;
        lateral_cost = 0;
        long_cost = 0;
        cost = 0;
    }
    ~OP_Lane(){

    }

    std::vector<WayPoint> waypoints;
    double transition_cost;
    double center_cost;
    double lateral_cost;
    double long_cost;
    double cost;
};


class TrajectoryGen
{
protected:
    PlannerH m_Planner;
    geometry_msgs::Pose m_OriginPos;
    WayPoint m_InitPos;
    bool bInitPos;

    WayPoint m_CurrentPos;
    bool bNewCurrentPos;
    bool bVehicleStatus;
    bool m_bUseMoveingObjectsPrediction;

    std::vector<std::vector<WayPoint>> generated_rollouts;
    std::vector<WayPoint> m_temp_path;
    std::vector<std::vector<WayPoint>> m_GlobalPaths;
    std::vector<std::vector<WayPoint>> m_GlobalPathSections;
    std::vector<WayPoint> t_centerTrajectorySmoothed;
    std::vector<std::vector<std::vector<WayPoint>>> m_RollOuts;
    bool bWayGlobalPath;
    struct timespec m_PlanningTimer;
    std::vector<std::string> m_LogData;
    PlanningParams m_PlanningParams;
    CAR_BASIC_INFO m_CarInfo;
    VehicleState m_VehicleStatus;
    TrajectoryDynamicCosts m_TrajectoryCostsCalculator;
    std::vector<DetectedObject> m_PredictedObjects;

    // ROS messages (topics)
    ros::NodeHandle nh;

    // define publishers
    ros::Publisher pub_LocalTrajectoriesRviz;

    // define subscribers.
    ros::Subscriber sub_initialpose;
    ros::Subscriber sub_current_pose;
    ros::Subscriber sub_current_velocity;
    ros::Subscriber sub_robot_odom;
    ros::Subscriber sub_can_info;
    ros::Subscriber sub_GlobalPlannerPaths;

    // Helper Functions
    void UpdatePlanningParams(ros::NodeHandle &_nh);

public:
    TrajectoryGen();
    ~TrajectoryGen();
    void TrajectoriesToMarkers(const std::vector<std::vector<std::vector<WayPoint>>> &paths, visualization_msgs::MarkerArray &markerArray);
    visualization_msgs::Marker TrajectoriesToMarker(const OP_Lane &paths);
    DiscretizedTrajectory MainLoop(const std::vector<std::vector<ReferencePoint>> reference_points, const std::array<double, 3>  &vehicle_pos,
                                   const double vehicle_speed, const std::vector<Obstacle> AllObstacles);
    void TransformWayPoint(const std::vector<ReferencePoint> reference_points_, std::vector<WayPoint> &t_centerTrajectorySmoothed_);
    double cast_from_PI_to_PI_Angle(const double &ang);
    std::vector<DetectedObject> TransformObject(const std::vector<Obstacle> Obstacles_);
};
