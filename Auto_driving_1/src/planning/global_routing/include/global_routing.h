#ifndef __GLOBAL_ROUTING_H
#define __GLOBAL_ROUTING_H

#include <iostream>
#include <ros/ros.h>
#include <boost/thread.hpp>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "std_msgs/String.h"
#include "ReferenceLine_Split.h"
#include "global_path_struct.h"
#include "stanley_controller.h"
#include "pure_pursuit_controller.h"
#include "lqr_controller.h"
#include "pid_controller.h"
#include <visualization_msgs/MarkerArray.h>

using namespace std;

class GlobalRouting
{
public:
  GlobalRouting();
  ~GlobalRouting();
  void thread_routing(void);

  void odom_call_back(const nav_msgs::Odometry &odom);
  void start_pose_call_back(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
  void goal_pose_call_back(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void get_Waypoints_From_Carla(const nav_msgs::Path &waypoints);

  void Vehival_Speed(const geometry_msgs::PoseArray &speed);
  void Vehival_Theta(const geometry_msgs::PoseArray &theta);
  void Vehival_Kappa(const geometry_msgs::PoseArray &kappa);
  void Vehival_Go(const std_msgs::String::ConstPtr &go);
  void Vehival_Traj(const nav_msgs::Path &Traj);
  bool Vehical_Stop(const geometry_msgs::Pose &goal_point, nav_msgs::Odometry &odom);

  void create_map();
  void publish_car_start_pose(const geometry_msgs::Pose &start_pose);

private:
  Eigen::MatrixXd hdmap_way_points;           // 中心点坐标
  std::vector<double> speeds, thetas, kappas; // 获取局部规划的速度与航向角,曲率
  nav_msgs::Odometry car_odom_;

  // 函数对象
  ReferenceLine_Split Rs;                       // 参考线生成
  geometry_msgs::Vector3 msg_ros;               // 发布控制数据
  visualization_msgs::MarkerArray obstacle_points, map_points;

  // 控制算法
  StanleyController Stanly;
  PursuitController Pursuit;
  LqrController Lqr;
  PidController Pid;

  // 路径点
  nav_msgs::Path dynamic_points;
  Eigen::MatrixXd hdmap_way_points_;

  // 点
  geometry_msgs::Pose start_pose_;
  geometry_msgs::Pose goal_pose_;
  geometry_msgs::Vector3 car_state;

  // visual Pub
  ros::Publisher obstacle_points_pub_;
  ros::Publisher goal_point_pub_;
  ros::Publisher map_points_pub_;
  ros::Publisher control_data_pub_;
  ros::Publisher vehicle_start_pose_pub_;
  // visual Subscriber
  ros::Subscriber odom_sub_;
  ros::Subscriber waypoints_sub_;
  ros::Subscriber start_pose_subscriber_;
  ros::Subscriber goal_pose_subscriber_;
  ros::Subscriber vehival_speed;
  ros::Subscriber vehival_theta;
  ros::Subscriber vehival_kappa;
  ros::Subscriber vehival_go;
  ros::Subscriber vehival_traj;
  // thread
  boost::thread *routing_thread_;

  // flag
  bool is_begin_reference;  // 控制参考线发布一次
  int obstacle_id;          // 手动标注障碍物位置坐标显示
  double car_speed;         // 车速度
  bool is_vehical_stop_set; // 判断车是否要停下来
  bool is_start_pose_set;   // 是否定义了起点
  bool is_goal_pose_set;    // 是否定义了终点
  int use_what_planner;     // 规划算法
  int use_what_controller;  // 控制算法
  string Start_dynamic;     // 判断局部规划是否开始
};

#endif