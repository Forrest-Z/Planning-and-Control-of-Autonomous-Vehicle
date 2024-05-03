#pragma once
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#define PI 3.1415926
using namespace std;

class StanleyController
{
public:
    StanleyController() = default;
    ~StanleyController() = default;

    int calTargetIndex(const nav_msgs::Odometry &robot_state, const nav_msgs::Path &trj_point);
    double normalizeAngle(double angle);

    void stanley_control(const nav_msgs::Odometry &robot_state, const nav_msgs::Path &trj_point,
                         const vector<double> trj_k, const vector<double> trj_t, const vector<double> trj_v,
                         double &out_turn_agl, int &out_index);
};