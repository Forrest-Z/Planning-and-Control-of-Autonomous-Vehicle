#pragma once

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/PoseStamped.h"
#include <tf/tf.h>

using namespace std;

class PursuitController
{
public:
    PursuitController();
    ~PursuitController() = default;

    int calTargetIndex(const nav_msgs::Odometry &robot_state, const nav_msgs::Path &trj_point, double l_d);

    void pure_pursuit_control(const nav_msgs::Path &trj_point_array, const nav_msgs::Odometry &currentWaypoint,
                              const double carVelocity, double &out_turn_agl, int &out_index);

private:
    int min_index;
    std::vector<float> r_x_;
    std::vector<float> r_y_;
};
