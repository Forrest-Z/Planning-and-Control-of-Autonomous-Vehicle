#include "global_routing.h"

/*定义起点位置的回调函数*/
void GlobalRouting::start_pose_call_back(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
  if (Start_dynamic == "0") // Start_dynamic == "0"代表车已经到达终点,即准备开始新的规划路线
  {
    is_start_pose_set = true;
    start_pose_.position.x = msg->pose.pose.position.x;
    start_pose_.position.y = msg->pose.pose.position.y;
    start_pose_.orientation = msg->pose.pose.orientation;

    std::cout << "start_x:" << msg->pose.pose.position.x << std::endl;
    std::cout << "start_y:" << msg->pose.pose.position.y << std::endl;
    std::cout << "theta_start:" << tf::getYaw(msg->pose.pose.orientation) << "\n";
    publish_car_start_pose(start_pose_);

    // 起点当作参考线起点
    // hdmap_way_points(0, 0) = start_pose_.position.x;
    // hdmap_way_points(0, 1) = start_pose_.position.y;
    // hdmap_way_points(0, 2) = 0;

    // 显示，方便
    visualization_msgs::Marker marker;
    marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
    marker.header.frame_id = Frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.id = obstacle_id++; // 注意了
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = start_pose_.position.x;
    marker.pose.position.y = start_pose_.position.y;
    marker.pose.position.z = 0;
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 0;
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();
    obstacle_points.markers.push_back(marker);
    obstacle_points_pub_.publish(obstacle_points);
  }

  else
  {
    ROS_WARN("ego vehicle is running!");
  }
}

/*定义终点位置的回调函数*/
void GlobalRouting::goal_pose_call_back(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  // Start_dynamic == "0"代表车已经到达终点,即准备开始新的规划路线
  if (Start_dynamic == "0" && is_start_pose_set == true)
  {
    is_goal_pose_set = true;
    goal_pose_.position.x = msg->pose.position.x;
    goal_pose_.position.y = msg->pose.position.y;
    goal_pose_.orientation = msg->pose.orientation;
  }
  else if (Start_dynamic == "0" && is_start_pose_set == false)
  {
    ROS_WARN("Please set the starting point!");
  }
  else if (Start_dynamic != "0")
  {
    ROS_WARN("ego vehicle is running!");
  }
}

/*读回carla全局规划发过来的导航点数据 , 作为参考线的路径点*/
void GlobalRouting::get_Waypoints_From_Carla(const nav_msgs::Path &waypoints)
{
  // 重置原来的参考路径点。
  hdmap_way_points = Eigen::MatrixXd::Zero(waypoints.poses.size(), 3);

  for (int i = 0; i < waypoints.poses.size(); i++)
  {
    hdmap_way_points(i, 0) = waypoints.poses[i].pose.position.x;
    hdmap_way_points(i, 1) = waypoints.poses[i].pose.position.y;
    hdmap_way_points(i, 2) = 0;
  }
}

/*读回ros odom坐标系数据 , 接收车的里程信息，控制车的移动*/
void GlobalRouting::odom_call_back(const nav_msgs::Odometry &odom)
{
  car_odom_ = odom; // 车的里程信息，就是位置信息
  // std::cout << "head:" << tf::getYaw(car_odom_.pose.pose.orientation) << "\n";
}

/*获取局部规划来的速度*/
void GlobalRouting::Vehival_Speed(const geometry_msgs::PoseArray &speed)
{
  speeds.clear();
  for (int i = 0; i < speed.poses.size(); i++)
  {
    double x = speed.poses[i].position.x;
    speeds.push_back(x);
  }
}

/*获取局部规划来的航向角*/
void GlobalRouting::Vehival_Theta(const geometry_msgs::PoseArray &theta)
{
  thetas.clear();
  if (theta.poses.size() > 0)
  {
    for (int i = 0; i < theta.poses.size(); i++)
    {
      double x = theta.poses[i].position.x;
      thetas.push_back(x);
    }
  }
}

/*获取局部规划来的曲率*/
void GlobalRouting::Vehival_Kappa(const geometry_msgs::PoseArray &kappa)
{
  kappas.clear();
  if (kappa.poses.size() > 0)
  {
    for (int i = 0; i < kappa.poses.size(); i++)
    {
      double x = kappa.poses[i].position.x;
      kappas.push_back(x);
    }
  }
}

/*获取局部轨迹的信号*/
void GlobalRouting::Vehival_Go(const std_msgs::String::ConstPtr &go)
{
  Start_dynamic = go->data.c_str();
}

/*获取局部轨迹*/
void GlobalRouting::Vehival_Traj(const nav_msgs::Path &Traj)
{
  dynamic_points.poses.clear();
  dynamic_points.header.frame_id = Frame_id;
  dynamic_points.header.stamp = ros::Time::now();
  for (int i = 0; i < Traj.poses.size(); i++)
  {
    geometry_msgs::PoseStamped pose_stamp;
    pose_stamp.header.frame_id = Frame_id;
    pose_stamp.header.stamp = ros::Time::now();
    pose_stamp.pose.position.x = Traj.poses[i].pose.position.x;
    pose_stamp.pose.position.y = Traj.poses[i].pose.position.y;
    pose_stamp.pose.position.z = 0;
    dynamic_points.poses.push_back(pose_stamp);
  }
}

/*发布车辆在路径的起始位置,传入起点*/
void GlobalRouting::publish_car_start_pose(const geometry_msgs::Pose &start_pose)
{
  // get 起始位置的yaw
  double set_vehicle_yaw = tf::getYaw(start_pose.orientation);
  // get vehicle start pose
  car_state.x = start_pose.position.x;
  car_state.y = start_pose.position.y;
  car_state.z = set_vehicle_yaw;
  // 发布车的起点位置
  vehicle_start_pose_pub_.publish(car_state);
}

// 判断车是否到了路径的尽头，传入 终点，车里程
bool GlobalRouting::Vehical_Stop(const geometry_msgs::Pose &goal_point, nav_msgs::Odometry &odom)
{
  // 获取车的里程位置
  double x = odom.pose.pose.position.x;
  double y = odom.pose.pose.position.y;
  // 获取路径点的最后一个位置
  double xt = goal_point.position.x;
  double yt = goal_point.position.y;
  // 判断如果两点坐标接近

  double dis = sqrt((x - xt) * (x - xt) + (y - yt) * (y - yt));

  if (dis < 1.0)
  {
    return true;
  }
  return false;
}

void GlobalRouting::create_map()
{
  // std::vector<double> map_1_x = {48.6505, 21.935, 0.32034, -4.89225, -8.76959, -14.8733, -20.663, -31.3403, -41.9258,
  //                                -48.5286, -54.6687, -59.3578, -61.0144, -61.6068, -60.3871, -57.5776, -52.5193};
  // std::vector<double> map_1_y = {-52.271, -27.4259, -7.07192, -2.08155, 1.55404, 7.22386, 12.8774, 22.9359, 33.0932,
  //                                39.2203, 45.6366, 51.9786, 57.4503, 63.7577, 82.018, 98.506, 125.59};

  // std::vector<double> map_1_x = {-41.9258,
  //                                -48.5286, -54.6687, -59.3578, -61.0144, -61.6068, -60.3871, -57.5776, -52.5193};
  // std::vector<double> map_1_y = {33.0932,
  //                                39.2203, 45.6366, 51.9786, 57.4503, 63.7577, 82.018, 98.506, 125.59};

  std::vector<double> map_1_x = {48.6505, 21.935, 0.32034, -8.76959};
  std::vector<double> map_1_y = {-52.271, -27.4259, -7.07192, 1.55404};

  // std::vector<double> map_1_x = {48.6505, 21.935};
  // std::vector<double> map_1_y = {-52.271, -27.4259};

  hdmap_way_points = Eigen::MatrixXd::Zero(map_1_x.size(), 3); // 初始化零矩阵
  for (int i = 0; i < map_1_x.size(); i++)
  {
    visualization_msgs::Marker marker;
    marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
    marker.header.frame_id = Frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.id = i; // 注意了
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = map_1_x[i];
    marker.pose.position.y = map_1_y[i];
    marker.pose.position.z = 0;
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 0;
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();
    map_points.markers.push_back(marker);
  }

  // 加入起点的话i = 1
  for (int i = 0; i < map_1_x.size(); i++)
  {
    hdmap_way_points(i, 0) = map_1_x[i];
    hdmap_way_points(i, 1) = map_1_y[i];
    hdmap_way_points(i, 2) = 0;
  }

  sleep(2);
  map_points_pub_.publish(map_points);
}

/*默认构造函数：规划函数,初始化参数*/
GlobalRouting::GlobalRouting()
{
  ros::NodeHandle n;
  // 参数获取
  n.param("use_what_planner", use_what_planner, 1);       // 局部轨迹生成方式选择
  n.param("use_what_controller", use_what_controller, 1); // 控制算法的选择

  start_pose_subscriber_ = n.subscribe("/initialpose", 10, &GlobalRouting::start_pose_call_back, this);
  goal_pose_subscriber_ = n.subscribe("/move_base_simple/goal", 10, &GlobalRouting::goal_pose_call_back, this);

  // 回调
  odom_sub_ = n.subscribe("/xsj/odom", 10, &GlobalRouting::odom_call_back, this);
  map_points_pub_ = n.advertise<visualization_msgs::MarkerArray>("/xsj/maps/map_points", 10);               // ros仿真下的地图路线点显示
  obstacle_points_pub_ = n.advertise<visualization_msgs::MarkerArray>("/xsj/obstacle/obstacle_points", 10); // 障碍物的顶点设置
  control_data_pub_ = n.advertise<geometry_msgs::Vector3>("/xsj/car/control_car", 10);                      // ros仿真时发布车的控制数据
  vehicle_start_pose_pub_ = n.advertise<geometry_msgs::Vector3>("/xsj/car/car_start_pose", 10);             // 发布车的起点位置

  // 局部规划订阅
  vehival_speed = n.subscribe("/xsj/planning/dynamic_paths_v", 100, &GlobalRouting::Vehival_Speed, this);
  vehival_theta = n.subscribe("/xsj/planning/dynamic_paths_t", 10, &GlobalRouting::Vehival_Theta, this);
  vehival_kappa = n.subscribe("/xsj/planning/dynamic_paths_k", 10, &GlobalRouting::Vehival_Kappa, this);
  vehival_traj = n.subscribe("/xsj/planning/dynamic_waypoints", 10, &GlobalRouting::Vehival_Traj, this);
  vehival_go = n.subscribe("/xsj/planning/Start_Dynamic", 10, &GlobalRouting::Vehival_Go, this);

  // 初始化 标志位：
  is_start_pose_set = false;
  is_goal_pose_set = false;
  is_begin_reference = false;
  is_vehical_stop_set = true;
  Start_dynamic = "0";
  obstacle_id = 0;

  // 打印控制器
  if (use_what_controller == 1)
    ROS_WARN("Controller: stanley");
  else if (use_what_controller == 2)
    ROS_WARN("Controller: lqr");
  else if (use_what_controller == 3)
    ROS_WARN("Controller: pure_pursuit");
  else if (use_what_controller == 4)
    ROS_WARN("Controller: pid");

  // 车的初始值设置
  car_speed = 0;

  // 创建map
  create_map();

  // 创建线程
  routing_thread_ = new boost::thread(boost::bind(&GlobalRouting::thread_routing, this));
}

/*析构函数：释放线程*/
GlobalRouting::~GlobalRouting()
{
  delete routing_thread_;
}

// 开车，控制车的行使
void GlobalRouting::thread_routing()
{
  ros::NodeHandle n;
  ros::Rate loop_rate(10);
  double turn_angle = 0;
  int out_index_ = 0;
  while (n.ok())
  {
    if (hdmap_way_points.rows() > 0 && is_start_pose_set == true &&
        is_goal_pose_set == true && is_begin_reference == false)
    {
      // 局部规划参考线:hdmap_way_points
      Rs.referenceLine_split(hdmap_way_points);
      is_begin_reference = true;
    }

    // is_start_pose_set== true已经设置起点
    // is_goal_pose_set== true已经设置终点
    // Start_dynamic 局部规划轨迹开始生成
    if (is_start_pose_set == true && is_goal_pose_set == true && Start_dynamic == "1")
    {
      if (use_what_controller == 1)
        Stanly.stanley_control(car_odom_, dynamic_points, kappas, thetas, speeds, turn_angle, out_index_);
      else if (use_what_controller == 2)
        Lqr.lqr_control(car_odom_, dynamic_points, kappas, thetas, speeds, turn_angle, out_index_);
      else if (use_what_controller == 3)
        Pursuit.pure_pursuit_control(dynamic_points, car_odom_, car_speed, turn_angle, out_index_);
      else if (use_what_controller == 4)
        Pid.pid_control(car_odom_, dynamic_points, kappas, thetas, speeds, turn_angle, out_index_);

      is_vehical_stop_set = Vehical_Stop(goal_pose_, car_odom_); // 判断是否到达终点
      msg_ros.y = turn_angle;                                    // 转角
      //  std::cout << "car_speed:" << car_speed << "\n";
      if (is_vehical_stop_set == false) // 未到达终点
      {
        car_speed = speeds[out_index_];
        msg_ros.x = car_speed; // ros仿真直接赋值速度
        control_data_pub_.publish(msg_ros);
      }
      else // 已经到达终点
      {
        msg_ros.x = 0;
        control_data_pub_.publish(msg_ros);
        is_start_pose_set = false;
        is_goal_pose_set = false;
        is_begin_reference = false;
        Start_dynamic = "0";
        car_speed = 0;
      }
    }
    else if (is_start_pose_set == true && is_goal_pose_set == true && Start_dynamic == "2") // 没有轨迹强制停车
    {
      msg_ros.x = 0;
      control_data_pub_.publish(msg_ros);
      car_speed = 0;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
}
