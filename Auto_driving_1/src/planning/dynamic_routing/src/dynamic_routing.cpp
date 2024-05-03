#include "dynamic_routing.h"

// 全局变量
std::vector<Obstacle> AllObstacle;                        // 感知一帧识别到的所有障碍物
std::pair<vector<double>, vector<double>> reference_path; // 参考路径点位置（x,y）
geometry_msgs::Pose start_pose_;                          // 起点
geometry_msgs::Pose goal_pose_;                           // 终点
nav_msgs::Odometry sub_odom_;                             // odom
bool messege1;                                            // 控制发布一次的变量
bool messege2;                                            // 控制发布一次的变量
double init_lon_state;                                    // 初始弧长
std::vector<double> accumulated_s;                        // 本参考线纵向距离
std::vector<ReferencePoint> reference_points;             // 本参考线参考路径点参数
std::vector<double> accumulated_s_other;                  // 另外一条参考线纵向距离
std::vector<ReferencePoint> reference_points_other;       // 另外一条参考线路径点参数
bool change_lane_sucessfully;                             // 是否变道成功
std::string referenceline_other = "1";                    // 选择其他车道的参考线编号

//-------------------------------------------回调函数---------------------------------------------//
/*定义起点位置*/
void Dynamic_routing::start_pose_call_backs(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
  if (ego_is_running == false)
  {
    // -------------------------路径规划参数在定义终点的时候初始化--------------------------//
    d0 = 0;                           // 初始的横向偏移值 [m]
    dd0 = 0;                          // 初始的横向速度 [m/s]
    ddd0 = 0;                         // 初始的横向加速度 [m/s^2]
    init_lon_state = 0;               // 初始的纵向值[m]
    ds0 = Config_.default_init_speed; // 初始的纵向速度[m/s]
    dds0 = 0;                         // 初始的纵向加速度[m/ss]
    init_relative_time = 0;           // 规划起始点的时间
    v_init = ds0;                     // 规划起始点的速度，没有横向速度，直接等于ds0
    a_init = 0;                       // 规划起始点的加速度
    z_init = 0;
    messege1 = false;

    start_pose_.position.x = msg->pose.pose.position.x;
    start_pose_.position.y = msg->pose.pose.position.y;
    start_pose_.orientation = msg->pose.pose.orientation;
    x_init = start_pose_.position.x;
    y_init = start_pose_.position.y;
    theta_init = tf2::getYaw(msg->pose.pose.orientation); // 初始朝向角度
  }
}

/*定义终点位置*/
void Dynamic_routing::goal_pose_call_backs(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  if (ego_is_running == false)
  {
    goal_pose_.position.x = msg->pose.position.x;
    goal_pose_.position.y = msg->pose.position.y;
    theta_end = tf2::getYaw(msg->pose.orientation);
  }
}

/*Hybrid map*/
void Dynamic_routing::setHybridMap(const nav_msgs::OccupancyGrid::Ptr map)
{
  grid = map;
  // create array for Voronoi diagram
  int height = map->info.height;
  int width = map->info.width;
  bool **binMap;
  binMap = new bool *[width];

  for (int x = 0; x < width; x++)
  {
    binMap[x] = new bool[height];
  }

  for (int x = 0; x < width; ++x)
  {
    for (int y = 0; y < height; ++y)
    {
      binMap[x][y] = map->data[y * width + x] ? true : false;
    }
  }
  voronoiDiagram.initializeMap(width, height, binMap);
  voronoiDiagram.update();
  voronoiDiagram.visualize();
}

/*订阅获取地图坐标*/
void Dynamic_routing::path_pose_call_backs(const nav_msgs::Path &path_point_)
{
  reference_path.first.clear();
  reference_path.second.clear();
  accumulated_s.clear();
  reference_points.clear();
  if (path_point_.poses.size() > 0)
  {
    std::vector<double> headings;
    std::vector<double> kappas;
    std::vector<double> dkappas;
    std::vector<std::pair<double, double>> xy_points;

    // auto beforeTime = std::chrono::steady_clock::now(); //计时开始
    for (size_t i = 0; i < path_point_.poses.size(); i++)
    {
      reference_path.first.push_back(path_point_.poses[i].pose.position.x);
      reference_path.second.push_back(path_point_.poses[i].pose.position.y);
      std::pair<double, double> xy;
      xy.first = path_point_.poses[i].pose.position.x;
      xy.second = path_point_.poses[i].pose.position.y;
      xy_points.push_back(xy);
    }
    // 这个过程需要消耗时间
    if (!PathMatcher::ComputePathProfile(xy_points, &headings, &accumulated_s, &kappas, &dkappas))
    {
      ROS_WARN("rerferenceline generate failed!");
    }
    else
    {
      for (size_t i = 0; i < xy_points.size(); i++)
      {
        // 创建ReferencePoint类
        ReferencePoint reference_point(kappas[i], dkappas[i], xy_points[i].first, xy_points[i].second, headings[i],
                                       accumulated_s[i]);
        reference_points.emplace_back(reference_point);
      }
      ROS_WARN("rerferenceline generate successfully!");
    }
    // 参考线三次样条方程
    // 输入(x1,y1),(x2,y2)......(xn,yn),返回 x-s坐标系与y-s坐标系
    csp = new CubicSpline2D(reference_path.first, reference_path.second, accumulated_s); // 只需要执行一次
    // 初始化
    kappa_init = kappas.front();
    dkappa_init = dkappas.front();
    init_lon_state = 0; // 复位
  }
  // 获取另外一条参考线参数
  MP.generate_reference_lines(referenceline_other, accumulated_s_other, reference_points_other);
  set_reference = true;
}

/*读回ros odom坐标系数据 , 接收车的里程信息，控制车的移动*/
void Dynamic_routing::odom_call_back(const nav_msgs::Odometry &odom)
{
  sub_odom_ = odom; // 车的里程信息，就是位置信息
  // std::cout << "car_odom_x:" << sub_odom_.pose.pose.position.x << "\n";
  // std::cout << "car_odom_y:" << sub_odom_.pose.pose.position.y << "\n";
  // std::cout << "car_odom_z:" << sub_odom_.pose.pose.position.z << "\n";
  // std::cout << "----------------------------------"
  //           << "\n";
}

//------------------------------------------子函数---------------------------------------------//
// 判断车是否走完轨迹的设置距离，是的话再发给局部规划，在车目前的位置之前更新局部规划轨迹
bool Dynamic_routing::is_update_dynamic(nav_msgs::Path &trj_point_array, nav_msgs::Odometry &odom, int size)
{
  double distance = sqrt(pow(odom.pose.pose.position.x - trj_point_array.poses[size].pose.position.x, 2) +
                         pow(odom.pose.pose.position.y - trj_point_array.poses[size].pose.position.y, 2));
  if (distance < 1) // 接近了
  {
    return true;
  }
  return false;
}

// 在frenet规划中，检查路径是否与障碍物碰撞,没有撞,返回false.撞了返回true （lattice规划用不到）
bool Dynamic_routing::is_collision(FrenetPath *fts, double COLLISION_CHECK)
{
  // 虚拟障碍物的避障
  for (size_t i = 0; i < AllObstacle.size(); i++) // 遍历障碍物
  {
    for (size_t j = 0; j < fts->x.size() - 1; j++) // 后半部分的点
    {
      double llx = AllObstacle[i].centerpoint.position.x;
      double lly = AllObstacle[i].centerpoint.position.y;
      double xx = llx - fts->x[j];
      double yy = lly - fts->y[j];
      double x2 = xx * xx;
      double y2 = yy * yy;
      double closest1 = sqrt(x2 + y2);
      if (closest1 <= COLLISION_CHECK) // 发现有点距离障碍物很近
      {
        return true;
        break;
      }
    }
  }
  return false;
}

//---------------------------------默认构造函数：规划函数,初始化参数---------------------------------//
Dynamic_routing::Dynamic_routing(void)
{
  ros::NodeHandle n;
  ros::param::get("use_what_planner", use_what_planner); // 选择规划方案
  ros::param::get("change_lane", change_lane);
  ros::param::get("use_lateral_optimization", FLAGS_lateral_optimization);
  ros::param::get("COLLISION_CHECK_THRESHOLD", COLLISION_CHECK_THRESHOLD);
  ros::param::get("MaxT", MaxT);
  ros::param::get("MinT", MinT);

  ////////////////////////////////发布////////////////////////////////////////
  waypoints_pub_ = n.advertise<nav_msgs::Path>("/xsj/planning/dynamic_waypoints", 10);        // 发布局部轨迹
  local_paths_v = n.advertise<geometry_msgs::PoseArray>("/xsj/planning/dynamic_paths_v", 10); // 发布局部轨迹的速度
  local_paths_a = n.advertise<geometry_msgs::PoseArray>("/xsj/planning/dynamic_paths_a", 10); // 发布局部轨迹的加速度
  local_paths_t = n.advertise<geometry_msgs::PoseArray>("/xsj/planning/dynamic_paths_t", 10); // 发布局部轨迹的航向角
  local_paths_k = n.advertise<geometry_msgs::PoseArray>("/xsj/planning/dynamic_paths_k", 10); // 发布局部轨迹的曲率
  Start_Dynamic = n.advertise<std_msgs::String>("/xsj/planning/Start_Dynamic", 10);           // 发布局部轨迹成功生存的信号

  other_reference_lines = n.advertise<nav_msgs::Path>("/xsj/reference/other_reference_lines", 10); // 发布所有参考线

  start_pose_subscriber_ = n.subscribe("/initialpose", 10, &Dynamic_routing::start_pose_call_backs, this);                             // 订阅
  goal_pose_subscriber_ = n.subscribe("/move_base_simple/goal", 10, &Dynamic_routing::goal_pose_call_backs, this);                     // 订阅
  path_pose_subscriber_ = n.subscribe("/xsj/reference/referenceLine_centerPoint", 1000, &Dynamic_routing::path_pose_call_backs, this); // 订阅

  odom_sub_ = n.subscribe("/xsj/odom", 10, &Dynamic_routing::odom_call_back, this);
  subMap = n.subscribe("/map", 1, &Dynamic_routing::setHybridMap, this);
  sleep(2);
  MP.visual_reference_lines(referenceline_other, reference_lines);
  other_reference_lines.publish(reference_lines); // 显示其他参考线
  // 参数初始化
  change_lane_sucessfully = false;
  ego_is_running = false;
  set_reference = false;
  string aa = "0";
  start_dynamic.data = aa.c_str();
  Start_Dynamic.publish(start_dynamic);

  // 打印规划器
  if (use_what_planner == 1)
    ROS_WARN("Planner: frenet based");
  else if (use_what_planner == 2)
    ROS_WARN("Planner: lattice");
  else if (use_what_planner == 3)
    ROS_WARN("Planner: em");
  else if (use_what_planner == 4)
    ROS_WARN("Planner: hyhid A*");
  else if (use_what_planner == 5)
    ROS_WARN("Planner: open planner");
  routing_thread_ = new boost::thread(boost::bind(&Dynamic_routing::thread_routing, this));
}

Dynamic_routing::~Dynamic_routing(void)
{
  delete routing_thread_;
  delete best_frenet_path;
  delete csp;
}

void Dynamic_routing::thread_routing(void)
{
  ros::NodeHandle n;
  ros::Rate loop_rate(20);
  // 类实例化对象
  FrenetOptimalTrajectory fot;
  GetObstacle gob;
  ObstacleTest obt; // 里面有线程，目的就是发布模拟障碍物
  Obstacle_avoid oba;
  // 前探距离
  double lon_decision_horizon = 0;

  // -------------------------Frenet采样路径规划参数初始化--------------------------//
  FrenetHyperparameters fot_hp = {
      10.8 / 3.6, // 最大速度 [m/s]，纵向和横向速度的尺量合
      2.0,        // 最大加速度[m/ss]
      1,          // 最大曲率 [1/m]
      2,          // 最大道路宽度 [m]
      -2,         // 最小道路宽度 [m]
      0.5,        // 道路宽度采样间隔 [m]，值越小，轨迹好的越多，但是太小计算效率不行
      0.2,        // 时间采样间隔[s]，值越大生成的轨迹速度越快
      MaxT,       // 最大预测时间 [s]，值越大生成的轨迹距离越远
      MinT,       // 最小预测时间 [s]，值越大生成的轨迹长度越长
      0.2,        // 目标速度采样间隔 [m/s]
      1,          // 目标速度的采样数量
      0.2,        // 目标位置采样间隔 [m/s]
      1,          // 目标位置的采样数量

      // 损失函数权重
      5.0, // Distance from reference path
      5.0, // Target speed
      1.0, // Target acceleration
      1.0, // Jerk
      0.5, // time
      // 总的
      1, // Lateral
      1, // Longitudin
  };

  while (n.ok())
  {
    if (set_reference == true)
    {
      //--------------------------------------------------生成轨迹--------------------------------------------------//
      if ((change_lane == true && reference_points_other.size() > 0) || (change_lane == false && reference_points.size() > 0))
      {
        // ---------------------------------------------RIO范围的纵向距离确定-----------------------------------------------//
        lon_decision_horizon = accumulated_s[accumulated_s.size() - 1]; // 我们参考线短，直接参考线的长度

        //--------------------------------------------------创建障碍物对象--------------------------------------------------//
        std::vector<const Obstacle *> obstacles; // Apollo是这种类型，不想改里面的源码，所以对外调整类型
        for (size_t i = 0; i < AllObstacle.size(); i++)
        {
          obstacles.emplace_back(&AllObstacle[i]);
        }
        // -----------------------------------------轨迹规划方案选择执行-------------------------------------------//
        // //车没有合适轨迹停下来重新规划的时候， 有些路径规划参数重新初始化
        if (messege2 == true)
        {
          // 有些路径规划参数重新初始化
          ds0 = Config_.default_init_speed; // 初始的纵向速度[m/s]
          init_relative_time = 0;           // 规划起始点的时间
          v_init = ds0;                     // 规划起始点的速度，没有横向速度，直接等于ds0
          x_init = sub_odom_.pose.pose.position.x;
          y_init = sub_odom_.pose.pose.position.y;
        }
        // auto beforeTime = std::chrono::steady_clock::now(); //计时开始

        if (use_what_planner == 1) // frenet采样规划 ， s/t和d/t
        {
          best_frenet_path = nullptr; // 确保已初始化最佳路径
          std::priority_queue<FrenetPath *, std::vector<FrenetPath *>, CostComparator_for_Frenet> frenet_paths;
          FrenetInitialConditions fot_ic = {
              init_lon_state,               // 初始的纵向值[m]
              ds0,                          // 初始的纵向速度[m/s]
              dds0,                         // 初始的纵向加速度[m/ss]
              d0,                           // 初始的横向偏移值 [m]
              dd0,                          // 初始的横向速度 [m/s]
              ddd0,                         // 初始的横向加速度 [m/s^2]
              0.0,                          // 目标横向速度配置
              0.0,                          // 目标横向加速度配置
              Config_.default_cruise_speed, // 目标速度（即纵向的速度保持） [m/s]
              20,                           // 目标位置,就是前探距离，测试3，按理说要变量赋值
              reference_path.first,         // 输入的参考线x坐标数组
              reference_path.second,        // 输入的参考线y坐标数组
          };
          // 计算轨迹，返回通过各种检测的轨迹数
          if (has_stop == false)
          {
            fot.calc_frenet_paths(&fot_ic, &fot_hp, frenet_paths, csp, reference_path.first, reference_path.second);
          }
          else if (has_stop == true)
          {
            fot.calc_frenet_paths_stop(&fot_ic, &fot_hp, frenet_paths, csp, reference_path.first, reference_path.second);
          }
          // 筛选最优轨迹
          while (!frenet_paths.empty())
          {
            auto top = frenet_paths.top();
            frenet_paths.pop();
            if (is_collision(top, COLLISION_CHECK_THRESHOLD) == false) // 通过碰撞检测
            {
              best_frenet_path = top;
              break;
            }
          }
          // 类型转换，确保发布一致性
          best_path = fot.FrenetPath_to_TrajectoryPoint(best_frenet_path);
        }
        else if (use_what_planner == 2) // lattice规划
        {
          // 创建起点参数
          InitialConditions planning_init_point = {
              d0,   // 初始的横向偏移值 [m]
              dd0,  // 初始的横向速度 [m/s]
              ddd0, // 初始的横向加速度 [m/s^2]

              init_lon_state,     // 初始的纵向值[m]
              ds0,                // 初始的纵向速度[m/s]
              dds0,               // 初始的纵向加速度[m/ss]
              init_relative_time, // 规划起始点的时间

              x_init,
              y_init,
              z_init,
              v_init,
              a_init,

              theta_init,
              kappa_init,
              dkappa_init,
          };

          std::pair<double, double> stop_s_l = MP.get_Stoppoint(goal_pose_, reference_points);

          // 变道决策
          if (change_lane == true) // 是否开启变道规划
          {
            std::array<double, 3> init_d_other = {0, 0, 0};
            auto ptr_path_time_graph_other = std::make_shared<PathTimeGraph>(obstacles, reference_points_other, 0,
                                                                             100, // 前瞻多少m
                                                                             0.0, Config_.FLAGS_trajectory_time_length, init_d_other);

            std::pair<double, double> ego_s_l = MP.get_EgoLonForReference_other(planning_init_point, reference_points_other);

            if (fabs(ego_s_l.second) < 0.8) // 变道成功
            {
              change_lane_sucessfully = true;
            }
            bool can_change_lane = false;
            if (change_lane_sucessfully == false)
            {
              // 获取是否可以变道
              can_change_lane = MP.CanChangeLaneToReference_other(ptr_path_time_graph_other, ego_s_l.first);
            }

            if (can_change_lane == true || change_lane_sucessfully == true)
            {
              PlanningTarget planning_target(Config_.default_cruise_speed, stop_s_l.first, accumulated_s_other); // 目标
              best_path = LP.LatticePlan(planning_init_point, planning_target, obstacles, accumulated_s_other, reference_points_other,
                                         FLAGS_lateral_optimization, init_relative_time, lon_decision_horizon);
            }
            else if (can_change_lane == false && change_lane_sucessfully == false)
            {
              PlanningTarget planning_target(Config_.default_cruise_speed, stop_s_l.first, accumulated_s); // 目标
              best_path = LP.LatticePlan(planning_init_point, planning_target, obstacles, accumulated_s, reference_points,
                                         FLAGS_lateral_optimization, init_relative_time, lon_decision_horizon);
            }
          }
          else
          {
            PlanningTarget planning_target(Config_.default_cruise_speed, stop_s_l.first, accumulated_s); // 目标
            if (sqrt((sub_odom_.pose.pose.position.x - goal_pose_.position.x) * (sub_odom_.pose.pose.position.x - goal_pose_.position.x) +
                     (sub_odom_.pose.pose.position.y - goal_pose_.position.y) * (sub_odom_.pose.pose.position.y - goal_pose_.position.y)) < 30.0)
            {
              planning_target.set_stop_point();
            }

            best_path = LP.LatticePlan(planning_init_point, planning_target, obstacles, accumulated_s, reference_points,
                                       FLAGS_lateral_optimization, init_relative_time, lon_decision_horizon);
          }
        }
        else if (use_what_planner == 3) // em planner规划
        {
          // 初始参数的输入
          InitialConditions init_point = {
              d0,   // 初始的横向偏移值 [m]
              dd0,  // 初始的横向速度 [m/s]
              ddd0, // 初始的横向加速度 [m/s^2]

              init_lon_state,     // 初始的纵向值[m]
              ds0,                // 初始的纵向速度[m/s]
              dds0,               // 初始的纵向加速度[m/ss]
              init_relative_time, // 规划起始点的时间

              sub_odom_.pose.pose.position.x, // sub_odom_.pose.pose.position.x,
              sub_odom_.pose.pose.position.y, // sub_odom_.pose.pose.position.y,
              z_init,
              v_init,
              a_init,

              theta_init,
              kappa_init,
              dkappa_init,
          };
          TrajectoryPoint planning_init_point;
          EM.CopyFrom(planning_init_point, init_point); // 有bug
          // 创建参考线对象
          ReferenceLine referenceLine(csp, reference_points, accumulated_s, reference_path);
          // 创建参考线info对象
          ReferenceLineInfo reference_line_info(init_point, referenceLine);
          // EM planner bug: 因为合成的轨迹长度用st决定，但是路径优化的长度比合成的长，所以不能实现到达终点停止
          best_path = EM.Plan(planning_init_point, referenceLine, reference_line_info, AllObstacle, obstacles, sub_odom_, theta_init, lon_decision_horizon);
        }
        else if (use_what_planner == 4) // Hybrid_a_star规划
        {
          double sx = start_pose_.position.x;
          double sy = start_pose_.position.y;
          double sphi = theta_init;
          double ex = goal_pose_.position.x;
          double ey = goal_pose_.position.y;
          double ephi = theta_end;
          std::vector<std::vector<Vec2d>> obstacles_list; // 所有障碍物的顶点
          for (auto &obstacle : AllObstacle)
          {
            obstacles_list.emplace_back(obstacle.polygon_points);
          }
          // 栅格图：100 X 100
          std::vector<double> XYbounds_;
          XYbounds_.push_back(sx - 100.0);
          XYbounds_.push_back(sx + 100.0);
          XYbounds_.push_back(sy - 100.0);
          XYbounds_.push_back(sy + 100.0);
          HybridAStartResult result;
          hybrid_test.Plan(sx, sy, sphi, ex, ey, ephi, XYbounds_, obstacles_list, &result);
          best_path = hybrid_test.HybridAStartResult_to_TrajectoryPoint(&result);
        }
        else if (use_what_planner == 5) // Open planner规划
        {
          std::array<double, 3> init_vehicle_pos = {x_init, y_init, theta_init}; // x,y,theta
          std::vector<std::vector<ReferencePoint>> ReferenceLines;
          ReferenceLines.emplace_back(reference_points);
          best_path = OP.MainLoop(ReferenceLines, init_vehicle_pos, v_init, AllObstacle);
        }
        else
        {
          ROS_WARN("No planning!");
        }

        // auto afterTime = std::chrono::steady_clock::now(); //计时结束
        // double duration_millsecond = std::chrono::duration<double, std::milli>(afterTime - beforeTime).count();
        // std::cout << duration_millsecond << "ms" << std::endl;
        // std::cout << "best_path:" << best_path.size() << "\n";

        // --------------------------------------最优轨迹的逻辑处理--------------------------------------//
        messege2 = false;
        if (best_path.size() == 0) // 没有轨迹数量
        {
          if (messege2 == false) // 只发布一次
          {
            ROS_WARN("vehical can not pass the road!");
            string aa = "2";
            start_dynamic.data = aa.c_str();
            Start_Dynamic.publish(start_dynamic); // 没有轨迹，发送停车信号
            // 发布空轨迹
            traj_points_.poses.clear();
            traj_points_.header.frame_id = Frame_id;
            traj_points_.header.stamp = ros::Time::now();
            waypoints_pub_.publish(traj_points_);
            messege1 = false; // 复位
            messege2 = true;  // 复位
          }
        }
        else
        {
          //---------------------------------发布轨迹---------------------------------//
          // 复制给traj_points_，给发布
          traj_points_.poses.clear();
          traj_points_.header.frame_id = Frame_id;
          traj_points_.header.stamp = ros::Time::now();
          for (int i = 0; i < best_path.size(); i++)
          {
            geometry_msgs::PoseStamped pose_stamp;
            pose_stamp.header.frame_id = Frame_id;
            pose_stamp.header.stamp = ros::Time::now();
            pose_stamp.pose.position.x = best_path[i].x;
            pose_stamp.pose.position.y = best_path[i].y;
            pose_stamp.pose.position.z = 0;
            traj_points_.poses.push_back(pose_stamp);
          }

          waypoints_pub_.publish(traj_points_);
          //--------------------------------发布速度---------------------------------//
          pubLocalPath_v.poses.clear();
          pubLocalPath_v.header.frame_id = Frame_id;
          pubLocalPath_v.header.stamp = ros::Time::now();
          for (size_t i = 0; i < best_path.size(); i++)
          {
            geometry_msgs::Pose init_pose;
            init_pose.position.x = best_path[i].v;
            pubLocalPath_v.poses.push_back(init_pose);
          }
          local_paths_v.publish(pubLocalPath_v);
          //--------------------------------发布加速度---------------------------------//
          pubLocalPath_a.poses.clear();
          pubLocalPath_a.header.frame_id = Frame_id;
          pubLocalPath_a.header.stamp = ros::Time::now();
          for (size_t i = 0; i < best_path.size(); i++)
          {
            geometry_msgs::Pose init_pose;
            init_pose.position.x = best_path[i].a;
            pubLocalPath_a.poses.push_back(init_pose);
          }
          local_paths_a.publish(pubLocalPath_a);
          //--------------------------------发布角度---------------------------------//
          pubLocalPath_t.poses.clear();
          pubLocalPath_t.header.frame_id = Frame_id;
          pubLocalPath_t.header.stamp = ros::Time::now();
          for (size_t i = 0; i < best_path.size(); i++)
          {
            geometry_msgs::Pose init_pose;
            init_pose.position.x = best_path[i].theta;
            pubLocalPath_t.poses.push_back(init_pose);
          }
          local_paths_t.publish(pubLocalPath_t);
          //--------------------------------发布曲率---------------------------------//
          pubLocalPath_k.poses.clear();
          pubLocalPath_k.header.frame_id = Frame_id;
          pubLocalPath_k.header.stamp = ros::Time::now();
          for (size_t i = 0; i < best_path.size(); i++)
          {
            geometry_msgs::Pose init_pose;
            init_pose.position.x = best_path[i].kappa;
            pubLocalPath_k.poses.push_back(init_pose);
          }
          local_paths_k.publish(pubLocalPath_k);

          //--------------------------------发布局部轨迹生成成功标志---------------------------------//
          if (messege1 == false) // 只发布一次,最开始生成局部轨迹的时候发
          {
            sleep(0.5); // 等会再开车，延时1秒
            string aa = "1";
            start_dynamic.data = aa.c_str();
            Start_Dynamic.publish(start_dynamic); // 发送局部轨迹生成信号给全局路径
            messege1 = true;
          }
          //--------------------------------更新轨迹，根据位置更新---------------------------------//
          ego_is_running = true;
          int update_pos;
          if (use_what_planner == 3)
            update_pos = 15;
          else if (use_what_planner == 5)
            update_pos = 3;
          else
            update_pos = 8;
          // use_what_planner != 4是因为混合A*不用更新，直接起点到终点规划（全局规划）
          if (is_update_dynamic(traj_points_, sub_odom_, update_pos + 3) == true && use_what_planner != 4)
          {
            init_lon_state = best_path[update_pos].s;
            ds0 = best_path[update_pos].s_d;
            dds0 = best_path[update_pos].s_dd;
            d0 = best_path[update_pos].d;
            dd0 = best_path[update_pos].d_d;
            ddd0 = best_path[update_pos].d_dd;

            init_relative_time = 0; // best_path[update_pos].relative_time，动态障碍物的起始时间跟这个保持一致
            x_init = best_path[update_pos].x;
            y_init = best_path[update_pos].y;
            z_init = 0;
            v_init = best_path[update_pos].v;
            a_init = best_path[update_pos].a;

            theta_init = best_path[update_pos].theta;
            kappa_init = best_path[update_pos].kappa;
            dkappa_init = best_path[update_pos].dkappa;
          }
        }
        // 到达终点判断
        if (sqrt((sub_odom_.pose.pose.position.x - goal_pose_.position.x) * (sub_odom_.pose.pose.position.x - goal_pose_.position.x) +
                 (sub_odom_.pose.pose.position.y - goal_pose_.position.y) * (sub_odom_.pose.pose.position.y - goal_pose_.position.y)) < 1.0)
        {
          string aa = "0";
          start_dynamic.data = aa.c_str();
          Start_Dynamic.publish(start_dynamic); // 发送局部轨迹生成信号给全局路径
          ego_is_running = false;
          set_reference = false;
        }
      }
      else
      {
        ROS_WARN("Other reference lines are empty and cannot plan!");
      }
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
}
