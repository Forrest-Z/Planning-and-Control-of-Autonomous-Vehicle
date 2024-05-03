/*
//基于Frenet坐标系的局部规划，参考论文：
Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame 2010_paper
author:xia
date:2020.12
*/
#include "FrenetOptimalTrajectory.h"

//构造函数
FrenetOptimalTrajectory::FrenetOptimalTrajectory()
{
}
//析构函数
FrenetOptimalTrajectory::~FrenetOptimalTrajectory()
{
}
/*
      计算frenet空间中的轨迹
       使用给定的启动动力学并生成轨迹变化
 */
void FrenetOptimalTrajectory::calc_frenet_paths(
    FrenetInitialConditions *fot_ic, FrenetHyperparameters *fot_hp,
    std::priority_queue<FrenetPath *, std::vector<FrenetPath *>, CostComparator_for_Frenet> &frenet_path,
    CubicSpline2D *csp, std::vector<double> x, std::vector<double> y)
{
  double t, ti, tv;
  double lateral_deviation, lateral_velocity, lateral_acceleration, lateral_jerk;
  double longitudinal_acceleration, longitudinal_jerk;
  double di;
  double ix_, iy_, iyaw_, dii, fx, fy, dx, dy, dyaw;
  //%调用类，FrenetPath里面定义了一系列参数
  //这里分两条写了。tfp要复制fp有的参数值，最后使用tfp就可以
  //生成每个偏移目标的路径，即d-t坐标系下的路径轨迹和s-t下的速度轨迹
  di = fot_hp->Min_road_width;
  while (di <= fot_hp->Max_road_width + fot_hp->D_road_w)
  {
    ti = fot_hp->Mint;
    // 横向规划 d/t坐标系
    while (ti <= fot_hp->Maxt + fot_hp->Dt)
    {
      //初始化用于累加的参数
      lateral_deviation = 0;
      lateral_velocity = 0;
      lateral_acceleration = 0;
      lateral_jerk = 0;
      FrenetPath *fp = new FrenetPath();
      // d0, dd0, ddd0为初始配置， di, ddT, dddT为目标配置
      //计算出关于目标配置di，Ti的横向多项式，5次多项式
      QuinticPolynomial lat_qp = QuinticPolynomial(fot_ic->d0, fot_ic->dd0, fot_ic->ddd0, di, 0, 0, ti);
      // 构建frenet路径，t从0开始，间隔DT,加到ti
      t = 0.0;
      while (t <= ti + fot_hp->Dt) //其实就是0-ti
      {
        //存储d/t坐标系的值
        fp->t.push_back(t);                                   //时间戳存储，这样写是生成数组
        fp->d.push_back(lat_qp.calc_point(t));                //存储横向偏移
        fp->d_d.push_back(lat_qp.calc_first_derivative(t));   //取一阶导数值
        fp->d_dd.push_back(lat_qp.calc_second_derivative(t)); //取二阶导数值
        fp->d_ddd.push_back(lat_qp.calc_third_derivative(t)); //取三阶导数值
        //累加，为了求代价函数
        lateral_deviation += fabs(lat_qp.calc_point(t));                //横向偏差累加
        lateral_velocity += fabs(lat_qp.calc_first_derivative(t));      //横向速度累加
        lateral_acceleration += fabs(lat_qp.calc_second_derivative(t)); //横向加速度累加
        lateral_jerk += fabs(lat_qp.calc_third_derivative(t));          //横向加加速度累加
        t += fot_hp->Dt;                                                //加Dt,循环
      }
      // 纵向速度规划 s/t坐标系
      tv = fot_ic->target_speed - fot_hp->D_t_s * fot_hp->N_s_sample;
      while (tv <= fot_ic->target_speed + fot_hp->D_t_s * fot_hp->N_s_sample + fot_hp->D_t_s)
      {
        longitudinal_acceleration = 0;
        longitudinal_jerk = 0;
        // copy frenet path
        FrenetPath *tfp = new FrenetPath();
        tfp->t.assign(fp->t.begin(), fp->t.end());
        tfp->d.assign(fp->d.begin(), fp->d.end());
        tfp->d_d.assign(fp->d_d.begin(), fp->d_d.end());
        tfp->d_dd.assign(fp->d_dd.begin(), fp->d_dd.end());
        tfp->d_ddd.assign(fp->d_ddd.begin(), fp->d_ddd.end());
        //四次多项式
        QuarticPolynomial lon_qp = QuarticPolynomial(fot_ic->s0, fot_ic->ds0, 0.0, tv, 0.0, ti);
        // 纵向规划
        for (double tp : tfp->t) //其实就是0-ti,tfp->t为数组，每个值间隔Dt
        {
          //存储s/t坐标系的值
          tfp->s.push_back(lon_qp.calc_point(tp));                //取值
          tfp->s_d.push_back(lon_qp.calc_first_derivative(tp));   //取一阶导数值
          tfp->s_dd.push_back(lon_qp.calc_second_derivative(tp)); //取二阶导数值
          tfp->s_ddd.push_back(lon_qp.calc_third_derivative(tp)); //取三阶导数值
          //累加，为了求代价函数
          longitudinal_acceleration += fabs(lon_qp.calc_second_derivative(tp));
          longitudinal_jerk += fabs(lon_qp.calc_third_derivative(tp));
        }

        // 失败或路径无效时删除
        //每次生成一条轨迹,就进行检测,检测不过之间删除并跳到下一个循环
        bool success = to_global_path(*tfp, csp, x, y);
        if (!success || !is_valid_path(*tfp))
        {
          // 释放内存并继续
          delete tfp;
          tv += fot_hp->D_t_s;
          continue;
        }

        //横向的代价，代价函数待改善
        tfp->c_lateral_deviation = lateral_deviation;
        tfp->c_lateral_velocity = lateral_velocity;
        tfp->c_lateral_acceleration = lateral_acceleration;
        tfp->c_lateral_jerk = lateral_jerk;
        tfp->Jd = fot_hp->kj * tfp->c_lateral_jerk + fot_hp->kd * tfp->c_lateral_deviation +
                  fot_hp->kv * tfp->c_lateral_velocity + fot_hp->kt * ti;
        //纵向的代价，代价函数待改善
        tfp->c_longitudinal_acceleration = longitudinal_acceleration;
        tfp->c_longitudinal_jerk = longitudinal_jerk;
        tfp->c_end_speed_deviation = pow((fot_ic->target_speed - tfp->s_d.back()), 2); //速度差
        tfp->c_time_taken = ti;
        tfp->Js = fot_hp->kj * tfp->c_longitudinal_jerk + fot_hp->kt * tfp->c_time_taken +
                  fot_hp->kd * tfp->c_end_speed_deviation + fot_hp->ka * tfp->c_longitudinal_acceleration;
        tfp->J = fot_hp->klat * tfp->Jd + fot_hp->klon * tfp->Js;
        frenet_path.emplace(tfp);
        tv += fot_hp->D_t_s;
      }
      ti += fot_hp->Dt;
      // 确保取消分配
      delete fp;
    }
    di += fot_hp->D_road_w;
  }
}

void FrenetOptimalTrajectory::calc_frenet_paths_stop(
    FrenetInitialConditions *fot_ic, FrenetHyperparameters *fot_hp,
    std::priority_queue<FrenetPath *, std::vector<FrenetPath *>, CostComparator_for_Frenet> &frenet_path,
    CubicSpline2D *csp, std::vector<double> x, std::vector<double> y)
{
  double t, ti, tv;
  double lateral_deviation, lateral_velocity, lateral_acceleration, lateral_jerk;
  double longitudinal_acceleration, longitudinal_jerk;
  double di;
  double ix_, iy_, iyaw_, dii, fx, fy, dx, dy, dyaw;
  ////%调用类，FrenetPath里面定义了一系列参数
  //这里分两条写了。tfp要复制fp有的参数值，最后使用tfp就可以
  //生成每个偏移目标的路径，即d-t坐标系下的路径轨迹和s-t下的速度轨迹
  di = fot_hp->Min_road_width;
  while (di <= fot_hp->Max_road_width + fot_hp->D_road_w)
  {
    ti = fot_hp->Mint;
    // 横向规划 d/t坐标系
    while (ti <= fot_hp->Maxt + fot_hp->Dt)
    {
      //初始化用于累加的参数
      lateral_deviation = 0;
      lateral_velocity = 0;
      lateral_acceleration = 0;
      lateral_jerk = 0;
      FrenetPath *fp = new FrenetPath();
      // d0, dd0, ddd0为初始配置， di, ddT, dddT为目标配置
      //计算出关于目标配置di，Ti的横向多项式，5次多项式
      QuinticPolynomial lat_qp = QuinticPolynomial(fot_ic->d0, fot_ic->dd0, fot_ic->ddd0, di, 0, 0, ti);
      // 构建frenet路径，t从0开始，间隔DT,加到ti
      t = 0.0;
      while (t <= ti + fot_hp->Dt) //其实就是0-ti
      {
        //存储d/t坐标系的值
        fp->t.push_back(t);                                   //时间戳存储，这样写是生成数组
        fp->d.push_back(lat_qp.calc_point(t));                //存储横向偏移
        fp->d_d.push_back(lat_qp.calc_first_derivative(t));   //取一阶导数值
        fp->d_dd.push_back(lat_qp.calc_second_derivative(t)); //取二阶导数值
        fp->d_ddd.push_back(lat_qp.calc_third_derivative(t)); //取三阶导数值
        //累加，为了求代价函数
        lateral_deviation += fabs(lat_qp.calc_point(t));                //横向偏差累加
        lateral_velocity += fabs(lat_qp.calc_first_derivative(t));      //横向速度累加
        lateral_acceleration += fabs(lat_qp.calc_second_derivative(t)); //横向加速度累加
        lateral_jerk += fabs(lat_qp.calc_third_derivative(t));          //横向加加速度累加
        t += fot_hp->Dt;                                                //加Dt,循环
      }
      // 纵向速度规划 s/t坐标系
      tv = fot_ic->target_position - fot_hp->D_s * fot_hp->N_s_position;
      while (tv <= fot_ic->target_position + fot_hp->D_s * fot_hp->N_s_position)
      {
        longitudinal_acceleration = 0;
        longitudinal_jerk = 0;
        // copy frenet path
        FrenetPath *tfp = new FrenetPath();
        tfp->t.assign(fp->t.begin(), fp->t.end());
        tfp->d.assign(fp->d.begin(), fp->d.end());
        tfp->d_d.assign(fp->d_d.begin(), fp->d_d.end());
        tfp->d_dd.assign(fp->d_dd.begin(), fp->d_dd.end());
        tfp->d_ddd.assign(fp->d_ddd.begin(), fp->d_ddd.end());
        //五次多项式
        QuinticPolynomial lon_qp = QuinticPolynomial(fot_ic->s0, fot_ic->ds0, fot_ic->dds0, tv, 0, 0, ti);

        // 纵向规划
        for (double tp : tfp->t) //其实就是0-ti,tfp->t为数组，每个值间隔Dt
        {
          //存储s/t坐标系的值
          tfp->s.push_back(lon_qp.calc_point(tp));              //取值
          tfp->s_d.push_back(lon_qp.calc_first_derivative(tp)); //取一阶导数值

          tfp->s_dd.push_back(lon_qp.calc_second_derivative(tp)); //取二阶导数值
          tfp->s_ddd.push_back(lon_qp.calc_third_derivative(tp)); //取三阶导数值
          //累加，为了求代价函数
          longitudinal_acceleration += fabs(lon_qp.calc_second_derivative(tp));
          longitudinal_jerk += fabs(lon_qp.calc_third_derivative(tp));
        }

        // 失败或路径无效时删除
        //每次生成一条轨迹,就进行检测,检测不过之间删除并跳到下一个循环
        bool success = to_global_path(*tfp, csp, x, y);
        if (!success)
        {
          // 释放内存并继续
          delete tfp;
          tv += fot_hp->D_t_s;
          continue;
        }
        //横向的代价，代价函数待改善
        tfp->c_lateral_deviation = lateral_deviation;
        tfp->c_lateral_velocity = lateral_velocity;
        tfp->c_lateral_acceleration = lateral_acceleration;
        tfp->c_lateral_jerk = lateral_jerk;
        tfp->Jd = fot_hp->kj * tfp->c_lateral_jerk + fot_hp->kd * tfp->c_lateral_deviation +
                  fot_hp->kv * tfp->c_lateral_velocity + fot_hp->kt * ti;
        //纵向的代价，代价函数待改善
        tfp->c_longitudinal_acceleration = longitudinal_acceleration;
        tfp->c_longitudinal_jerk = longitudinal_jerk;
        tfp->c_end_speed_deviation = pow((fot_ic->target_position - tfp->s.back()), 2); //速度差
        tfp->c_time_taken = ti;
        tfp->Js = fot_hp->kj * tfp->c_longitudinal_jerk + fot_hp->kt * tfp->c_time_taken +
                  fot_hp->kd * tfp->c_end_speed_deviation;
                  
        tfp->J = fot_hp->klat * tfp->Jd + fot_hp->klon * tfp->Js;
        frenet_path.emplace(tfp);
        tv += fot_hp->D_t_s;
      }
      ti += fot_hp->Dt;
      // 确保取消分配
      delete fp;
    }
    di += fot_hp->D_road_w;
  }
}

// Convert the frenet path to global path in terms of x, y, yaw, velocity
bool FrenetOptimalTrajectory::to_global_path(FrenetPath &fts, CubicSpline2D *csp, std::vector<double> &x,
                                             std::vector<double> &y)
{
  double ix_, iy_, iyaw_, di, fx, fy, dx, dy;
  // 计算世界坐标系下的位置x,y
  for (size_t i = 0; i < fts.s.size(); i++)
  {
    ix_ = csp->calc_x(fts.s[i]);
    iy_ = csp->calc_y(fts.s[i]);
    if (isnan(ix_) || isnan(iy_)) //如果ix_,iy_不存在
      break;
    iyaw_ = csp->calc_yaw(fts.s[i]);
    fts.ix.push_back(ix_);
    fts.iy.push_back(iy_);
    fts.iyaw.push_back(iyaw_);
    di = fts.d[i];
    fx = ix_ + di * cos(iyaw_ + M_PI_2);
    fy = iy_ + di * sin(iyaw_ + M_PI_2);
    fts.x.push_back(fx);
    fts.y.push_back(fy);
    fts.v.push_back(fts.s_d[i]);
    fts.a.push_back(fts.s_dd[i]);
  }
  // //没有足够的点来构造有效路径
  if (fts.x.size() <= 1)
  {
    return false;
  }
  // 计算 theta and dL (running length)
  for (size_t i = 0; i < fts.x.size() - 1; i++)
  {
    dx = fts.x[i + 1] - fts.x[i];
    dy = fts.y[i + 1] - fts.y[i];
    fts.theta.push_back(atan2(dy, dx)); //航向角
    fts.dL.push_back(hypot(dx, dy));    //垂直里程
  }
  fts.theta.push_back(fts.theta.back()); //.back()返回vetor最末尾的值
  fts.dL.push_back(fts.dL.back());
  // 计算曲率
  fts.kappa = Calculate_curvature(fts);

  return true;
}

// 检查轨迹是否可行（要满足最大条件）,以及碰撞检测
bool FrenetOptimalTrajectory::is_valid_path(FrenetPath &fts)
{
  // max speed check
  if (any_of(fts.s_d.begin(), fts.s_d.end(), [this](int i)
             { return fabs(i) > 3; }))
  {
    return false;
  }
  // max accel check
  else if (any_of(fts.s_dd.begin(), fts.s_dd.end(), [this](int i)
                  { return fabs(i) > 2; }))
  {
    return false;
  }
  // max curvature check
  else if (fts.kappa > 0.1979)
  {
    return false;
  }
  return true;
}
//三点求一条轨迹的曲率
double FrenetOptimalTrajectory::Calculate_curvature(FrenetPath fts)
{
  double curvity;
  //取点 P1、P2、P3是位置信息
  //即一条轨迹的起点，中间点和终点
  Point2D P1, P2, P3;
  P1.x = fts.x[0];
  P1.y = fts.y[0];
  int n = fts.x.size() / 2;
  P2.x = fts.x[n];
  P2.y = fts.y[n];
  P3.x = fts.x[fts.x.size() - 1];
  P3.y = fts.y[fts.y.size() - 1];

  double k1 = (P2.y - P1.y) / (P2.x - P1.x);
  double k2 = (P3.y - P1.y) / (P3.x - P1.x);
  //计算曲率部分，使用正弦定理 a/sinA = 2R
  if (fabs(k1 - k2) <= 0.01) //三点共线，直接标记曲率为0
  {
    curvity = 0;
  }
  else
  {
    double dis1, dis2, dis3;
    double cosA, sinA, dis;
    dis1 = sqrt((P1.x - P2.x) * (P1.x - P2.x) + (P1.y - P2.y) * (P1.y - P2.y));
    dis2 = sqrt((P1.x - P3.x) * (P1.x - P3.x) + (P1.y - P3.y) * (P1.y - P3.y));
    dis3 = sqrt((P2.x - P3.x) * (P2.x - P3.x) + (P2.y - P3.y) * (P2.y - P3.y));
    dis = dis1 * dis1 + dis3 * dis3 - dis2 * dis2;
    cosA = dis / (2 * dis1 * dis3); //余弦定理求角度
    sinA = sqrt(1 - cosA * cosA);   //求正弦
    curvity = 0.5 * dis2 / sinA;    //正弦定理求外接圆半径
    curvity = 1 / curvity;          //半径的倒数是曲率，半径越小曲率越大
  }
  return curvity;
}

DiscretizedTrajectory FrenetOptimalTrajectory::FrenetPath_to_TrajectoryPoint(FrenetPath *best_path)
{
  DiscretizedTrajectory Optim_trajectory;
  if (best_path == nullptr)
    return {};

  for (int i = 0; i < best_path->x.size(); ++i)
  {
    TrajectoryPoint trajectory_point;
    //赋值frenet坐标系下的
    trajectory_point.set_relative_time(best_path->t[i]);
    trajectory_point.set_s(best_path->s[i]);
    trajectory_point.s_d = best_path->s_d[i];
    trajectory_point.s_dd = best_path->s_dd[i];
    trajectory_point.d = best_path->d[i];
    trajectory_point.d_d = best_path->d_d[i];
    trajectory_point.d_dd = best_path->d_dd[i];
    //赋值世界坐标系下的
    trajectory_point.set_x(best_path->x[i]);
    trajectory_point.set_y(best_path->y[i]);
    trajectory_point.set_theta(best_path->theta[i]);
    trajectory_point.set_kappa(0);
    trajectory_point.set_v(best_path->v[i]);
    trajectory_point.set_a(best_path->a[i]);

    Optim_trajectory.emplace_back(trajectory_point);
  }

  return Optim_trajectory;
}