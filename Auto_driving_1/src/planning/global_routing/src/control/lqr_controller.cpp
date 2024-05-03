/*
鸣谢: 本文件代码参考了https://blog.csdn.net/weixin_42301220/article/details/125031348
*/

#include "lqr_controller.h"

/**
 * 解代数里卡提方程
 * @param A 状态矩阵A
 * @param B 状态矩阵B
 * @param Q Q为半正定的状态加权矩阵, 通常取为对角阵；Q矩阵元素变大意味着希望跟踪偏差能够快速趋近于零；
 * @param R R为正定的控制加权矩阵，R矩阵元素变大意味着希望控制输入能够尽可能小。
 * @return
 */
MatrixXd LqrController::calRicatti(MatrixXd A, MatrixXd B, MatrixXd Q, MatrixXd R)
{
    MatrixXd Qf = Q;
    MatrixXd P = Qf;
    MatrixXd P_;
    for (int i = 0; i < 100; i++) // 迭代次数为100，可设置
    {
        P_ = Q + A.transpose() * P * A - A.transpose() * P * B * (R + B.transpose() * P * B).inverse() * B.transpose() * P * A;
        // 小于预设精度时返回
        if ((P_ - P).maxCoeff() < EPS && (P - P_).maxCoeff() < EPS)
            break;
        P = P_;
    }
    return P_;
}

/**
 * 角度归一化
 * @param angle
 * @return
 */
double LqrController::normalizeAngle(double angle)
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
 * Lqr控制器
 * @param robot_state 车状态，位置和朝向，速度
 * @param trj_point 轨迹点位置信息
 * @param trj_k  轨迹点曲率信息
 * @param trj_t  轨迹点朝向信息
 * @param trj_v  轨迹点速度信息
 * @param out_turn_agl 输出转角
 * @param out_index 输出最接近轨迹点的index
 */
void LqrController::lqr_control(const nav_msgs::Odometry &robot_state,  const nav_msgs::Path &trj_point,
                                const vector<double> trj_k, const vector<double> trj_t, const vector<double> trj_v,
                                double &out_turn_agl, int &out_index)
{
    // wheel_base 车轮轴距，可调节
    double wheel_base = 2.7;

    // 1.求出x,y,phi
    double x = robot_state.pose.pose.position.x;
    double y = robot_state.pose.pose.position.y;
    double phi = tf::getYaw(robot_state.pose.pose.orientation);

    // 2.求出距离
    vector<double> d_x(trj_point.poses.size()), d_y(trj_point.poses.size()), d(trj_point.poses.size());
    for (int i = 0; i < trj_point.poses.size(); i++)
    {
        d_x[i] = trj_point.poses[i].pose.position.x - x;
        d_y[i] = trj_point.poses[i].pose.position.y - y;
        d[i] = sqrt(d_x[i] * d_x[i] + d_y[i] * d_y[i]);
    }

    // 3.求出 error,k,yaw,min_index
    int min_index = min_element(d.begin(), d.end()) - d.begin();
    double k = trj_k[min_index];
    double yaw = trj_t[min_index];
    double angle = normalizeAngle(yaw - atan2(d_y[min_index], d_x[min_index]));
    double error = d[min_index]; // 误差
    if (angle < 0)
        error *= -1;

    // 4.定义矩阵
    double dt = 0.1;
    double robot_v = trj_v[min_index];
    MatrixXd A(3, 3), B(3, 2);
    A << 1.0, 0.0, -robot_v * dt * sin(yaw),
        0.0, 1.0, robot_v * dt * cos(yaw),
        0.0, 0.0, 1.0;
    B << dt * cos(yaw), 0,
        dt * sin(yaw), 0,
        dt * tan(atan2(wheel_base * k, 1)) / wheel_base, robot_v * dt / (wheel_base * cos(atan2(wheel_base * k, 1)) * cos(atan2(wheel_base * k, 1)));

    MatrixXd Q(3, 3);
    Q << 3, 0, 0,
        0, 3, 0,
        0, 0, 3; // 可调节
    MatrixXd R(2, 2);
    R << 2.0, 0.0,
        0.0, 2; // 可调节

    // 5.求解
    MatrixXd X(3, 1);
    X << x - trj_point.poses[min_index].pose.position.x, y - trj_point.poses[min_index].pose.position.y, phi - trj_t[min_index];
    MatrixXd P = calRicatti(A, B, Q, R);
    MatrixXd K = -(R + B.transpose() * P * B).inverse() * B.transpose() * P * A;
    MatrixXd u = K * X;

    out_turn_agl = u(1, 0);
    out_index = min_index;
}