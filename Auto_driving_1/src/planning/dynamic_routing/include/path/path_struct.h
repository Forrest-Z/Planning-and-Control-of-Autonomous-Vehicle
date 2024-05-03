#ifndef FRENETOPTIMALTRAJECTORY_PY_CPP_STRUCT_H
#define FRENETOPTIMALTRAJECTORY_PY_CPP_STRUCT_H
#include "QuarticPolynomial.h"
#include "QuinticPolynomial.h"
#include "curve1d.h"
#include "quintic_polynomial_curve1d.h"
#include "quartic_polynomial_curve1d.h"
#include "lattice_trajectory1d.h"
#include <nav_msgs/Odometry.h>
#include "Configs.h"

//一旦将对象定义为常对象之后，不管是哪种形式，该对象就只能访问被 const 修饰的成员了（包括 const 成员变量和 const 成员函数
const std::string Frame_id = "map"; //参考系的定义
const Param_Configs Config_;             //所有参数

//单条轨迹
typedef std::vector<std::shared_ptr<Curve1d>> Trajectory1DBundle;

//优先级队列，fist：存放配对的纵向和横向轨迹，second:存放总的cost
typedef std::pair<std::pair<std::shared_ptr<Curve1d>, std::shared_ptr<Curve1d>>, double> PairCost;

//防函数
struct CostComparator : public std::binary_function<const PairCost &, const PairCost &, bool>
{
  //返回true时，a的优先级低于b的优先级（a排在b的后面）
  bool operator()(const PairCost &left, const PairCost &right) const
  {
    return left.second > right.second;
  }
};

///////////////////////////////Frennet_dynamic/////////////////////////////////////
//输入的初始化参数
struct FrenetInitialConditions
{
  double s0;   //初始的纵向值[m]
  double ds0;  //初始的纵向速度[m/s]
  double dds0; //初始的纵向加速度[m/ss]

  double d0;   //初始的横向偏移值 [m]
  double dd0;  //初始的横向速度 [m/s]
  double ddd0; //初始的横向加速度 [m/s^2]

  double ddT;             //目标横向速度配置
  double dddT;            //目标横向加速度配置
  double target_speed;    //目标速度（即纵向的速度保持） [m/s]
  double target_position; // 目标速度的采样数量
  std::vector<double> wx; //输入的x坐标数组
  std::vector<double> wy; //输入的y坐标数组
};
//需要用到的参数
struct FrenetHyperparameters
{
  //参数
  double Max_speed;      //最大速度 [m/s]，纵向和横向速度的尺量合
  double Max_accel;      //最大加速度[m/ss]
  double Max_curvature;  // 最大曲率 [1/m]
  double Max_road_width; // 最大道路宽度 [m]
  double Min_road_width; //最小道路宽度 [m]
  double D_road_w;       //道路宽度采样间隔 [m]
  double Dt;             // 时间采样间隔[s]
  double Maxt;           // 最大预测时间 [s]
  double Mint;           //最小预测时间 [s]
  double D_t_s;          //目标速度采样间隔 [m/s]
  double N_s_sample;     // 目标速度的采样数量
  double D_s;            //目标位置采样间隔 [m]
  double N_s_position;   // 目标位置的采样数量

  //损失函数权重
  double kd; // Distance from reference path
  double kv; // Target speed
  double ka; // Target acceleration
  double kj; // Jerk
  double kt; // time

  //总的
  double klat; // Lateral
  double klon; // Longitudinal
};

struct Point2D
{
  double x;
  double y;
};

///////////////////////////////Lattice_dynamic and EM_dynamic//////////////////////////////////////
struct InitialConditions
{
  double d0;   //初始的横向偏移值 [m]
  double dd0;  //初始的横向速度 [m/s]
  double ddd0; //初始的横向加速度 [m/s^2]

  double s0;   //初始的纵向值[m]
  double ds0;  //初始的纵向速度[m/s]
  double dds0; //初始的纵向加速度[m/ss]

  double init_relative_time; //规划起始点的时间

  double x_init;
  double y_init;
  double z_init;

  double v_init;
  double a_init;

  double theta_init;
  double kappa_init;
  double dkappa_init;
};

/////////////////////////////////Vehicle Config/////////////////////////////////////
class VehicleConfig
{
public:
  VehicleConfig() = default;
  VehicleConfig(const float vehicle_width_, const float vehicle_length_, const float vehicle_velocity_,
                const nav_msgs::Odometry vehicle_pos_)
      : vehicle_width(vehicle_width_), vehicle_length(vehicle_length_), vehicle_velocity(vehicle_velocity_), vehicle_pos(vehicle_pos_)
  {
  }

  ~VehicleConfig() = default;

  float vehicle_width;
  float vehicle_length;
  float vehicle_velocity;
  nav_msgs::Odometry vehicle_pos;
};

#endif