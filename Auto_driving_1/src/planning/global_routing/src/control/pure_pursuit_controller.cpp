/*
鸣谢: 本文件代码参考了https://blog.csdn.net/weixin_42301220/article/details/124882144?spm=1001.2014.3001.5501
*/

#include "pure_pursuit_controller.h"

PursuitController::PursuitController()
{
    min_index = -1;
}
/**
 * Pursuit控制器
 * @param robot_state 车状态，位置和朝向，速度
 * @param trj_point 轨迹点位置信息
 * @param trj_k  轨迹点曲率信息
 * @param trj_t  轨迹点朝向信息
 * @param trj_v  轨迹点速度信息
 * @param out_turn_agl 输出转角
 * @param out_index 输出最接近轨迹点的index
 */
void PursuitController::pure_pursuit_control(const nav_msgs::Path &trj_point_array, const nav_msgs::Odometry &currentWaypoint, const double carVelocity, double &out_turn_agl, int &out_index)
{
    int pointNum = 0; // 保存路径点的个数
    int targetIndex = pointNum - 1;
    vector<float> bestPoints_;

    float PREVIEW_DIS = 1;
    float preview_dis = 0;
    float k = 0.1;
    preview_dis = k * carVelocity + PREVIEW_DIS;

    pointNum = trj_point_array.poses.size();
    for (int i = 0; i < pointNum; i++)
    {
        r_x_.push_back(trj_point_array.poses[i].pose.position.x);
        r_y_.push_back(trj_point_array.poses[i].pose.position.y);
    }

    auto currentPositionX = currentWaypoint.pose.pose.position.x;
    auto currentPositionY = currentWaypoint.pose.pose.position.y;

    double car_yaw = tf::getYaw(currentWaypoint.pose.pose.orientation);

    // 通过计算当前坐标和目标轨迹距离，找到一个最小距离的索引号
    int index;
    for (int i = 0; i < pointNum; i++)
    {
        // float lad = 0.0;
        float path_x = r_x_[i];
        float path_y = r_y_[i];
        // 遍历所有路径点和当前位置的距离，保存到数组中
        float lad = sqrt(pow(path_x - currentPositionX, 2) +
                         pow(path_y - currentPositionY, 2));

        bestPoints_.push_back(lad);
    }
    // 找到数组中最小横向距离
    auto smallest = min_element(bestPoints_.begin(), bestPoints_.end());
    // 找到最小横向距离的索引位置
    index = distance(bestPoints_.begin(), smallest);

    int temp_index;
    for (int i = index; i < pointNum; i++)
    {
        // 遍历路径点和预瞄点的距离，从最小横向位置的索引开始
        float dis = sqrt(pow(r_y_[index] - r_y_[i], 2) + pow(r_x_[index] - r_x_[i], 2));
        // 判断跟预瞄点的距离
        if (dis < preview_dis)
        {
            temp_index = i;
        }
        else
        {
            break;
        }
    }
    index = temp_index;
    float alpha = atan2(r_y_[index] - currentPositionY, r_x_[index] - currentPositionX) - car_yaw;

    // 当前点和目标点的距离Id
    float dl = sqrt(pow(r_y_[index] - currentPositionY, 2) +
                    pow(r_x_[index] - currentPositionX, 2));

    // 发布小车运动指令及运动轨迹
    float wheel_base = 2.7;
    out_turn_agl = atan(1 * wheel_base * sin(alpha) / dl);
    out_index = index;
}
