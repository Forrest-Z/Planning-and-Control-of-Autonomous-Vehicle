#pragma once
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

using namespace std;
#define PI 3.1415926

/**
 * 位置式PID实现
 */
class PidController
{
private:
    double error = 0.0, pre_error = 0.0, sum_error = 0.0;

public:
    PidController() = default;
    ~PidController() = default;

    int calTargetIndex(const nav_msgs::Odometry &robot_state, const nav_msgs::Path &trj_point);

    void pid_control(const nav_msgs::Odometry &robot_state, const nav_msgs::Path &trj_point,
                              const vector<double> trj_k, const vector<double> trj_t, const vector<double> trj_v,
                              double &out_turn_agl, int &out_index);
};
