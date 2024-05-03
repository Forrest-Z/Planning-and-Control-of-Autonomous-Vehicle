/*
鸣谢: 本文件代码参考了https://blog.csdn.net/weixin_42301220/article/details/124899547
*/ 

#include "stanley_controller.h"

/**
 * 搜索目标邻近路点
 * @param robot_state 当前机器人位置
 * @param refer_path 参考轨迹（数组）
 * @return
 */
int StanleyController::calTargetIndex(const nav_msgs::Odometry &robot_state, const nav_msgs::Path &trj_point)
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
 * 角度归一化到【-PI,PI】
 * @param angle
 * @return
 */
double StanleyController::normalizeAngle(double angle)
{
    while (angle > PI)
    {
        angle -= 2.0 * PI;
    }
    while (angle < -PI)
    {
        angle += 2.0 * PI;
    }
    return angle;
}

/**
 * Stanley控制器
 * @param robot_state 车状态，位置和朝向，速度
 * @param trj_point 轨迹点位置信息
 * @param trj_k  轨迹点曲率信息
 * @param trj_t  轨迹点朝向信息
 * @param trj_v  轨迹点速度信息
 * @param out_turn_agl 输出转角
 * @param out_index 输出最接近轨迹点的index
 * @return
 */
void StanleyController::stanley_control(const nav_msgs::Odometry &robot_state, const nav_msgs::Path &trj_point,
                                        const vector<double> trj_k, const vector<double> trj_t, const vector<double> trj_v,
                                        double &out_turn_agl, int &out_index)
{
    double k = 0.8; // 增益系数，可调节

    int current_target_index = calTargetIndex(robot_state, trj_point);

    // 当计算出来的目标临近点索引大于等于参考轨迹上的最后一个点索引时
    if (current_target_index >= trj_point.poses.size())
    {
        current_target_index = trj_point.poses.size() - 1;
    }

    double e_y;
    double psi_t = trj_t[current_target_index];

    // 计算横向误差e_y
    if ((robot_state.pose.pose.position.x - trj_point.poses[current_target_index].pose.position.x) * psi_t -
            (robot_state.pose.pose.position.y - trj_point.poses[current_target_index].pose.position.y) >
        0)
    {
        e_y = sqrt(pow(trj_point.poses[current_target_index].pose.position.x - robot_state.pose.pose.position.x, 2) +
                   pow(trj_point.poses[current_target_index].pose.position.y - robot_state.pose.pose.position.y, 2));
    }
    else
    {
        e_y = -sqrt(pow(trj_point.poses[current_target_index].pose.position.x - robot_state.pose.pose.position.x, 2) +
                    pow(trj_point.poses[current_target_index].pose.position.y - robot_state.pose.pose.position.y, 2));
    }
    // # 通过公式(5)计算转角,符号保持一致
    double psi = tf::getYaw(robot_state.pose.pose.orientation);
    double v = trj_v[current_target_index];
    double theta_e = psi_t - psi;
    double delta_e = atan2(k * e_y, v);
    double delta = normalizeAngle(delta_e + theta_e);

    out_turn_agl = delta;
    out_index = current_target_index;
}