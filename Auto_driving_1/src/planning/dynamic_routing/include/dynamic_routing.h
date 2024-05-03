#ifndef LATTICE_DYNAMIC_ROUTING_H
#define LATTICE_DYNAMIC_ROUTING_H

#include "CubicSpline2D.h"
#include "lattice_planner.h"
#include "em_planner.h"
#include "PlanningTarget.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include <string>
#include <vector>
#include "FrenetOptimalTrajectory.h"
#include "reference_line.h"
#include "Obstacle_test.h"
#include <nav_msgs/OccupancyGrid.h>
#include "dynamicvoronoi.h"
#include "hybrid_a_star.h"
#include "op_trajectory_generator.h"
#include "multilane.h"

using std::string;
using namespace std;

class Dynamic_routing
{
public:
  Dynamic_routing(void);
  ~Dynamic_routing(void);
  void thread_routing(void);
  void odom_call_back(const nav_msgs::Odometry &odom);
  void start_pose_call_backs(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
  void goal_pose_call_backs(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void setHybridMap(const nav_msgs::OccupancyGrid::Ptr map);
  void path_pose_call_backs(const nav_msgs::Path &path_point_);
  bool is_update_dynamic(nav_msgs::Path &trj_point_array, nav_msgs::Odometry &odom, int size);
  bool is_collision(FrenetPath *fts, double COLLISION_CHECK);
  void publish_vehical_start_pose(const geometry_msgs::Pose &start_pose);

private:
  int use_what_planner; // 选择规划方案
  bool change_lane;     // 变道决策
  double COLLISION_CHECK_THRESHOLD;
  double MaxT;
  double MinT;
  bool has_stop = false;           // 是否需要停车,先默认不需要
  bool FLAGS_lateral_optimization; // 选择二次规划
  bool set_reference;              // 判断参考线已经回调
  bool ego_is_running;

  // ROS
  // visual publish
  ros::Publisher waypoints_pub_;
  ros::Publisher waypoints_vis_pub_;
  ros::Publisher local_paths_v;
  ros::Publisher local_paths_s;
  ros::Publisher local_paths_a;
  ros::Publisher local_paths_t;
  ros::Publisher local_paths_k;
  ros::Publisher Start_Dynamic;
  ros::Publisher other_reference_lines;

  // visual sub
  ros::Subscriber odom_sub_;
  ros::Subscriber start_pose_subscriber_;
  ros::Subscriber goal_pose_subscriber_;
  ros::Subscriber path_pose_subscriber_;
  ros::Subscriber location_pose_subscriber_;
  ros::Subscriber subMap;

  // visualmsgs
  visualization_msgs::MarkerArray ObstacleArray;
  visualization_msgs::Marker marker;
  geometry_msgs::PoseArray pubLocalPath_v;
  geometry_msgs::PoseArray pubLocalPath_a;
  geometry_msgs::PoseArray pubLocalPath_s;
  geometry_msgs::PoseArray pubLocalPath_t;
  geometry_msgs::PoseArray pubLocalPath_k;
  nav_msgs::Path reference_lines;

  // 变量
  nav_msgs::Path traj_points_; // 局部规划的轨迹,nav_msgs类型
  double d0;                   // 初始的横向偏移值 [m]
  double dd0;                  // 初始的横向速度 [m/s]
  double ddd0;                 // 初始的横向加速度 [m/s^2]
  double init_lon_state;       // 初始的纵向值[m]
  double ds0;                  // 初始的纵向速度[m/s]
  double dds0;                 // 初始的纵向加速度[m/ss]
  double init_relative_time;   // 规划起始点的时间
  double x_init;
  double y_init;
  double z_init;
  double v_init;
  double a_init;
  double theta_init;
  double theta_end;
  double kappa_init;
  double dkappa_init;

  std_msgs::String start_dynamic;  // 判断局部规划是否开始
  DiscretizedTrajectory best_path; // 最佳路径
  FrenetPath *best_frenet_path;    // 最佳路径,要析构
  CubicSpline2D *csp;              // 样条插值函数
  DynamicVoronoi voronoiDiagram;   // The voronoi diagram

  nav_msgs::OccupancyGrid::Ptr grid;
  // visual sub
  ros::Subscriber odom_sub;
  ros::Subscriber start_pose_subscriber;
  ros::Subscriber goal_pose_subscriber;
  ros::Subscriber path_pose_subscriber;
  // thread
  boost::thread *routing_thread_;
  // 对象
  LatticePlanner LP;
  EMPlanner EM;
  HybridAStar hybrid_test;
  TrajectoryGen OP;
  MultilanePlanner MP;
};

#endif
