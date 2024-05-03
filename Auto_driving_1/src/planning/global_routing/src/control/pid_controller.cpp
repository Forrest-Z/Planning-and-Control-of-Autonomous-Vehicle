/*
鸣谢: 本文件代码参考了https://blog.csdn.net/weixin_42301220/article/details/124793474
*/

#include "pid_controller.h"

/**
 * 搜索目标邻近路点
 * @param robot_state 当前机器人位置
 * @param refer_path 参考轨迹（数组）
 * @return
 */
int PidController::calTargetIndex(const nav_msgs::Odometry &robot_state, const nav_msgs::Path &trj_point)
{
    vector<double> dists;
    for (int i = 0; i < trj_point.poses.size(); i++)
    {
        double dist = sqrt(pow(trj_point.poses[i].pose.position.x - robot_state.pose.pose.position.x, 2) +
                           pow(trj_point.poses[i].pose.position.y - robot_state.pose.pose.position.y, 2));
        dists.push_back(dist);
    }
    return min_element(dists.begin(), dists.end()) - dists.begin(); // 返回vector最小元素的下标
}

/**
 * Pid控制器
 * @param robot_state 车状态，位置和朝向，速度
 * @param trj_point 轨迹点位置信息
 * @param trj_k  轨迹点曲率信息
 * @param trj_t  轨迹点朝向信息
 * @param trj_v  轨迹点速度信息
 * @param out_turn_agl 输出转角
 * @param out_index 输出最接近轨迹点的index
 */
void PidController::pid_control(const nav_msgs::Odometry &robot_state, const nav_msgs::Path &trj_point,
                                         const vector<double> trj_k, const vector<double> trj_t, const vector<double> trj_v,
                                         double &out_turn_agl, int &out_index)

{
    double kp = 0.05, ki = 0.01, kd = 0.50, target = 0., upper = PI / 6, lower = -PI / 6;

    double min_ind = calTargetIndex(robot_state, trj_point);
    double alpha = atan2(trj_point.poses[min_ind].pose.position.y - robot_state.pose.pose.position.y,
                         trj_point.poses[min_ind].pose.position.x - robot_state.pose.pose.position.x);

    double l_d = sqrt(pow(trj_point.poses[min_ind].pose.position.x - robot_state.pose.pose.position.x, 2) +
                      pow(trj_point.poses[min_ind].pose.position.y - robot_state.pose.pose.position.y, 2));
    double theta_e = alpha - trj_t[min_ind];
    double e_y = -l_d * sin(theta_e);

    error = target - e_y;
    double u = error * kp + sum_error * ki + (error - pre_error) * kd;
    if (u < lower)
        u = lower;
    else if (u > upper)
        u = upper;
    pre_error = error;
    sum_error = sum_error + error;
    out_turn_agl = u;
    out_index = min_ind;
}
