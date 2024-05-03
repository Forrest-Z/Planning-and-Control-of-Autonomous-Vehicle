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
#include "op_trajectory_generator.h"

TrajectoryGen::TrajectoryGen()
{
    bInitPos = false;
    bNewCurrentPos = false;
    bVehicleStatus = false;
    bWayGlobalPath = false;
    m_bUseMoveingObjectsPrediction = false;
    ros::NodeHandle _nh;
    UpdatePlanningParams(_nh);
    pub_LocalTrajectoriesRviz = nh.advertise<visualization_msgs::MarkerArray>("/xsj/planning/open_planner/local_trajectories_gen_rviz", 1);
}

TrajectoryGen::~TrajectoryGen()
{
}

void TrajectoryGen::UpdatePlanningParams(ros::NodeHandle &_nh)
{
    _nh.getParam("/dynamic_routing/samplingTipMargin", m_PlanningParams.carTipMargin);
    _nh.getParam("/dynamic_routing/samplingOutMargin", m_PlanningParams.rollInMargin);
    _nh.getParam("/dynamic_routing/samplingSpeedFactor", m_PlanningParams.rollInSpeedFactor);
    _nh.getParam("/dynamic_routing/enableHeadingSmoothing", m_PlanningParams.enableHeadingSmoothing);
    _nh.getParam("/dynamic_routing/enablePrediction", m_bUseMoveingObjectsPrediction);

    _nh.getParam("/dynamic_routing/enableSwerving", m_PlanningParams.enableSwerving);
    if (m_PlanningParams.enableSwerving)
        m_PlanningParams.enableFollowing = true;
    else
        _nh.getParam("/dynamic_routing/enableFollowing", m_PlanningParams.enableFollowing);

    _nh.getParam("/dynamic_routing/enableTrafficLightBehavior", m_PlanningParams.enableTrafficLightBehavior);
    _nh.getParam("/dynamic_routing/enableStopSignBehavior", m_PlanningParams.enableStopSignBehavior);

    _nh.getParam("/dynamic_routing/maxVelocity", m_PlanningParams.maxSpeed);
    _nh.getParam("/dynamic_routing/minVelocity", m_PlanningParams.minSpeed);
    _nh.getParam("/dynamic_routing/maxLocalPlanDistance", m_PlanningParams.microPlanDistance);

    _nh.getParam("/dynamic_routing/pathDensity", m_PlanningParams.pathDensity);
    _nh.getParam("/dynamic_routing/rollOutDensity", m_PlanningParams.rollOutDensity);
    if (m_PlanningParams.enableSwerving)
        _nh.getParam("/dynamic_routing/rollOutsNumber", m_PlanningParams.rollOutNumber);
    else
        m_PlanningParams.rollOutNumber = 0;

    _nh.getParam("/dynamic_routing/horizonDistance", m_PlanningParams.horizonDistance);
    _nh.getParam("/dynamic_routing/minFollowingDistance", m_PlanningParams.minFollowingDistance);
    _nh.getParam("/dynamic_routing/minDistanceToAvoid", m_PlanningParams.minDistanceToAvoid);
    _nh.getParam("/dynamic_routing/maxDistanceToAvoid", m_PlanningParams.maxDistanceToAvoid);
    _nh.getParam("/dynamic_routing/speedProfileFactor", m_PlanningParams.speedProfileFactor);

    _nh.getParam("/dynamic_routing/smoothingDataWeight", m_PlanningParams.smoothingDataWeight);
    _nh.getParam("/dynamic_routing/smoothingSmoothWeight", m_PlanningParams.smoothingSmoothWeight);

    _nh.getParam("/dynamic_routing/horizontalSafetyDistance", m_PlanningParams.horizontalSafetyDistancel);
    _nh.getParam("/dynamic_routing/verticalSafetyDistance", m_PlanningParams.verticalSafetyDistance);

    _nh.getParam("/dynamic_routing/enableLaneChange", m_PlanningParams.enableLaneChange);

    _nh.getParam("/dynamic_routing/width", m_CarInfo.width);
    _nh.getParam("/dynamic_routing/length", m_CarInfo.length);
    _nh.getParam("/dynamic_routing/wheelBaseLength", m_CarInfo.wheel_base);
    _nh.getParam("/dynamic_routing/turningRadius", m_CarInfo.turning_radius);
    _nh.getParam("/dynamic_routing/maxSteerAngle", m_CarInfo.max_steer_angle);
    _nh.getParam("/dynamic_routing/maxAcceleration", m_CarInfo.max_acceleration);
    _nh.getParam("/dynamic_routing/maxDeceleration", m_CarInfo.max_deceleration);

    m_CarInfo.max_speed_forward = m_PlanningParams.maxSpeed;
    m_CarInfo.min_speed_forward = m_PlanningParams.minSpeed;
}

// 原理看：https://blog.csdn.net/Travis_X/article/details/115350840
DiscretizedTrajectory TrajectoryGen::MainLoop(const std::vector<std::vector<ReferencePoint>> reference_points, const std::array<double, 3>  &vehicle_pos,
                                              const double vehicle_speed, const std::vector<Obstacle> AllObstacles)
{
    DiscretizedTrajectory best_path;

    // 1.转化参考线数据
    m_GlobalPathSections.clear();
    // std::vector<ReferencePoint> 转 std::vector<std::vector<WayPoint>> ，按照我的理解：
    // 最外层容器表示参考线的数量，内层容器就是每条参考线上参考路径的参数WayPoint。但是，目前我们只有一条参考线
    for (size_t i = 0; i < reference_points.size(); i++) // 遍历每一条参考线
    {
        t_centerTrajectorySmoothed.clear();
        TransformWayPoint(reference_points[i], t_centerTrajectorySmoothed);
        m_GlobalPathSections.push_back(t_centerTrajectorySmoothed);
    }
    // 2.转化车信息
    m_CurrentPos.pos.x = vehicle_pos[0];
    m_CurrentPos.pos.y = vehicle_pos[1];
    m_CurrentPos.pos.z = 0;
    m_CurrentPos.pos.a = vehicle_pos[2];
    m_CurrentPos.v = vehicle_speed;

    m_VehicleStatus.speed = vehicle_speed;
    m_VehicleStatus.steer = vehicle_pos[2];

    // 3.Roll-outs Generator产生轨迹
    // m_RollOuts有三维度，分别为：参考线数量，每条参考线产生的RollOuts轨迹数量，每条RollOuts轨迹的轨迹点数量
    std::vector<WayPoint> sampledPoints_debug;
    m_Planner.GenerateRunoffTrajectory(m_GlobalPathSections,                     // 参考线
                                       m_CurrentPos,                             // 自主车的实时位置
                                       m_PlanningParams.enableLaneChange,        // 是否换道
                                       vehicle_speed,                            // 自主车的速度
                                       m_PlanningParams.microPlanDistance,       // 前探距离
                                       m_PlanningParams.maxSpeed,                // 最大速度
                                       m_PlanningParams.minSpeed,                // 最小速度
                                       m_PlanningParams.carTipMargin,            // car Tip
                                       m_PlanningParams.rollInMargin,            // rollIn
                                       m_PlanningParams.rollInSpeedFactor,       // rollInSpeedFactor
                                       m_PlanningParams.pathDensity,             // rollOut
                                       m_PlanningParams.rollOutDensity,          // rollOut
                                       m_PlanningParams.rollOutNumber,           // rollOut
                                       m_PlanningParams.smoothingDataWeight,     // 权重
                                       m_PlanningParams.smoothingSmoothWeight,   // 权重
                                       m_PlanningParams.smoothingToleranceError, // smoothingToleranceError
                                       m_PlanningParams.speedProfileFactor,
                                       m_PlanningParams.enableHeadingSmoothing, // 是否允许朝向平滑
                                       -1,
                                       -1,
                                       m_RollOuts, sampledPoints_debug);
    // 4.提取所有轨迹
    generated_rollouts.clear();
    for (size_t k = 0; k < m_RollOuts.size(); k++)
    {
        for (size_t m = 0; m < m_RollOuts[k].size(); m++)
        {
            // PlanningHelpers::PredictConstantTimeCostForTrajectory(m_RollOuts.at(k).at(m), m_CurrentPos, m_PlanningParams.minSpeed, m_PlanningParams.microPlanDistance);
            std::vector<WayPoint> rollout_singleLane;
            for (int h = 0; h < m_RollOuts[k][m].size(); h++)
            {
                WayPoint wp;
                wp.pos.x = m_RollOuts[k][m][h].pos.x;
                wp.pos.y = m_RollOuts[k][m][h].pos.y;
                wp.pos.z = m_RollOuts[k][m][h].pos.z;
                wp.pos.a = cast_from_PI_to_PI_Angle(m_RollOuts[k][m][h].pos.a);
                rollout_singleLane.push_back(wp);
            }

            generated_rollouts.push_back(rollout_singleLane);
        }
    }

    // 5.评价函数
    OP_TrajectoryCost best_trajectory;
    m_PredictedObjects = TransformObject(AllObstacles);
    if (m_bUseMoveingObjectsPrediction) // 适应动态障碍物
        best_trajectory = m_TrajectoryCostsCalculator.DoOneStepDynamic(generated_rollouts,
                                                                       m_GlobalPathSections.at(0),
                                                                       m_CurrentPos,
                                                                       m_PlanningParams,
                                                                       m_CarInfo,
                                                                       m_VehicleStatus,
                                                                       m_PredictedObjects,
                                                                       0); // m_CurrentBehavior.iTrajectory
    else
        best_trajectory = m_TrajectoryCostsCalculator.DoOneStepStatic(generated_rollouts,
                                                                      m_GlobalPathSections.at(0),
                                                                      m_CurrentPos,
                                                                      m_PlanningParams,
                                                                      m_CarInfo,
                                                                      m_VehicleStatus,
                                                                      m_PredictedObjects);
    // 6.提取最优轨迹
    double pre_index = -1;
    double pre_pre_index = -1;
    // double best_index = floor(0.6 * best_trajectory.index + 0.25 * pre_index + 0.15 * pre_pre_index); // floor：向下取整
    double best_index = best_trajectory.index;

    OP_Lane best_local_trajectory;
    for (int m = 0; m < generated_rollouts.size(); m++) // 遍历所有轨迹
    {
        OP_Lane lane_msg;
        lane_msg.transition_cost = m_TrajectoryCostsCalculator.m_TrajectoryCosts[m].transition_cost;
        lane_msg.center_cost = m_TrajectoryCostsCalculator.m_TrajectoryCosts[m].priority_cost;
        lane_msg.lateral_cost = m_TrajectoryCostsCalculator.m_TrajectoryCosts[m].lateral_cost;
        lane_msg.long_cost = m_TrajectoryCostsCalculator.m_TrajectoryCosts[m].longitudinal_cost;
        lane_msg.cost = m_TrajectoryCostsCalculator.m_TrajectoryCosts[m].cost;

        if (m == best_index)
        {
            best_local_trajectory = lane_msg;
            for (size_t ii = 0; ii < generated_rollouts[m].size(); ii++) // 遍历轨迹点
            {
                WayPoint p;
                p.pos.x = generated_rollouts[m][ii].pos.x;
                p.pos.y = generated_rollouts[m][ii].pos.y;
                p.pos.z = generated_rollouts[m][ii].pos.z;
                p.pos.a = generated_rollouts[m][ii].pos.a; // 朝向
                // p.v = generated_rollouts[m][ii].v;
                p.v = 2;
                best_local_trajectory.waypoints.push_back(p);
            }
        }
    }
    // std::cout << "best_local_trajectory:" << best_local_trajectory.waypoints.size() << "\n";

    // 将最优的轨迹输出
    for (size_t m = 0; m < best_local_trajectory.waypoints.size(); m++)
    {
        // std::cout << "v:" << best_local_trajectory.waypoints[m].v << "\n";

        TrajectoryPoint local_lane;
        local_lane.set_x(best_local_trajectory.waypoints[m].pos.x);
        local_lane.set_y(best_local_trajectory.waypoints[m].pos.y);
        local_lane.set_z(best_local_trajectory.waypoints[m].pos.z);

        local_lane.set_theta(best_local_trajectory.waypoints[m].pos.a);
        local_lane.set_v(best_local_trajectory.waypoints[m].v);

        best_path.push_back(local_lane);
    }

    // 调试将所有Roll-outs产生的轨迹显示
    visualization_msgs::MarkerArray all_rollOuts;
    TrajectoriesToMarkers(m_RollOuts, all_rollOuts);
    pub_LocalTrajectoriesRviz.publish(all_rollOuts);

    return best_path;
}

visualization_msgs::Marker TrajectoryGen::TrajectoriesToMarker(const OP_Lane &paths)
{
    visualization_msgs::Marker lane_waypoint_marker;
    lane_waypoint_marker.header.frame_id = Frame_id;
    lane_waypoint_marker.header.stamp = ros::Time();
    lane_waypoint_marker.ns = "lane_marker";
    lane_waypoint_marker.type = visualization_msgs::Marker::LINE_STRIP;
    lane_waypoint_marker.action = visualization_msgs::Marker::ADD;
    lane_waypoint_marker.scale.x = 0.1;
    lane_waypoint_marker.scale.y = 0.1;
    lane_waypoint_marker.frame_locked = false;

    lane_waypoint_marker.points.clear();
    lane_waypoint_marker.id = 0;

    for (unsigned int j = 0; j < paths.waypoints.size(); j++)
    {
        geometry_msgs::Point point;

        point.x = paths.waypoints[j].pos.x;
        point.y = paths.waypoints[j].pos.y;
        point.z = paths.waypoints[j].pos.z;

        lane_waypoint_marker.points.push_back(point);
    }

    lane_waypoint_marker.color.a = 0.9;
    lane_waypoint_marker.color.r = 1.0;
    lane_waypoint_marker.color.g = 0.0;
    lane_waypoint_marker.color.b = 0.0;

    return lane_waypoint_marker;
}

void TrajectoryGen::TrajectoriesToMarkers(const std::vector<std::vector<std::vector<WayPoint>>> &paths, visualization_msgs::MarkerArray &markerArray)
{
    visualization_msgs::Marker lane_waypoint_marker;
    lane_waypoint_marker.header.frame_id = Frame_id;
    lane_waypoint_marker.header.stamp = ros::Time();
    lane_waypoint_marker.ns = "global_lane_array_marker";
    lane_waypoint_marker.type = visualization_msgs::Marker::LINE_STRIP;
    lane_waypoint_marker.action = visualization_msgs::Marker::ADD;
    lane_waypoint_marker.scale.x = 0.1;
    lane_waypoint_marker.scale.y = 0.1;
    // lane_waypoint_marker.scale.z = 0.1;
    lane_waypoint_marker.frame_locked = false;
    std_msgs::ColorRGBA total_color, curr_color;

    int count = 0;
    for (unsigned int il = 0; il < paths.size(); il++)
    {
        for (unsigned int i = 0; i < paths.at(il).size(); i++)
        {
            lane_waypoint_marker.points.clear();
            lane_waypoint_marker.id = count;

            for (unsigned int j = 0; j < paths.at(il).at(i).size(); j++)
            {
                geometry_msgs::Point point;

                point.x = paths.at(il).at(i).at(j).pos.x;
                point.y = paths.at(il).at(i).at(j).pos.y;
                point.z = paths.at(il).at(i).at(j).pos.z;

                lane_waypoint_marker.points.push_back(point);
            }

            lane_waypoint_marker.color.a = 0.9;

            lane_waypoint_marker.color.r = 0.0;
            lane_waypoint_marker.color.g = 1.0;
            lane_waypoint_marker.color.b = 0.0;

            markerArray.markers.push_back(lane_waypoint_marker);
            count++;
        }
    }
    // std::cout << "count:" << count << "\n";
}

// 参考线用到pos信息的x,y,z,a(yaw)信息。
void TrajectoryGen::TransformWayPoint(const std::vector<ReferencePoint> reference_points_,
                                      std::vector<WayPoint> &t_centerTrajectorySmoothed_)
{
    for (size_t i = 0; i < reference_points_.size() - 5; i = i + 5) // 遍历每条参考线每一个参考路径点
    {
        WayPoint p;
        p.pos.x = reference_points_[i].x_;
        p.pos.y = reference_points_[i].y_;
        p.pos.z = 0; // 地图当屏幕
        p.pos.a = reference_points_[i].heading_;
        t_centerTrajectorySmoothed_.push_back(p);
    }
}

double TrajectoryGen::cast_from_PI_to_PI_Angle(const double &ang)
{
    double angle = 0;
    if (ang < -2.0 * M_PI || ang > 2.0 * M_PI)
    {
        angle = fmod(ang, 2.0 * M_PI);
    }
    else
        angle = ang;

    if (angle < -M_PI)
    {
        angle += 2.0 * M_PI;
    }
    else if (angle > M_PI)
    {
        angle -= 2.0 * M_PI;
    }
    return angle;
}

// Obstacle转为DetectedObject
std::vector<DetectedObject> TrajectoryGen::TransformObject(const std::vector<Obstacle> Obstacles_)
{
    std::vector<DetectedObject> res;
    for (auto &obstacle : Obstacles_)
    {
        DetectedObject ob;
        std::vector<GpsPoint> contour_;
        ob.label = obstacle.obstacle_id;
        ob.center.v = obstacle.obstacle_velocity;
        ob.center.pos.x = obstacle.centerpoint.position.x;
        ob.center.pos.y = obstacle.centerpoint.position.y;
        ob.l = obstacle.obstacle_length;
        ob.w = obstacle.obstacle_width;
        ob.h = obstacle.obstacle_height;
        // 遍历顶点
        for (size_t i = 0; i < obstacle.polygon_points.size(); i++)
        {
            GpsPoint gp;
            gp.x = obstacle.polygon_points[i].x();
            gp.y = obstacle.polygon_points[i].y();
            gp.z = 0;
            contour_.push_back(gp);
        }
        ob.contour = contour_;

        res.push_back(ob);
    }

    return res;
}
