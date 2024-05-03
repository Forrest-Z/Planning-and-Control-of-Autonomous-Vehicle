#include "Obstacle_test.h"

ObstacleTest::ObstacleTest()
{
  ros::NodeHandle nh_;
  Pub_Obstacles = nh_.advertise<object_msgs::DynamicObjectArray>("/xsj/obstacle/obstacles", 1);
  Obstacle_Visualization = nh_.advertise<visualization_msgs::MarkerArray>("/xsj/obstacle/vehicle_obstacles", 10);

  routing_thread_obstacle_test = new boost::thread(boost::bind(&ObstacleTest::thread_obstacle_test, this));
  routing_thread_obstacle = new boost::thread(boost::bind(&ObstacleTest::thread_obstacle_visualization, this));
}
ObstacleTest::~ObstacleTest()
{
}

// 输入：位置和朝向
void ObstacleTest::Publish_DynamicObstacle(double x_pose_covariance, double y_pose_covariance, double head,
                                           double x_pose_covariance1, double y_pose_covariance1, double head1)
{
  // 求障碍物顶点,往障碍物的运动方向看
  PPoint left_front;
  PPoint left_buttom;
  PPoint right_front;
  PPoint right_buttom;

  object_msgs::DynamicObject Object;
  Objects.objects.clear();
  /////////////////////////////////////////////////////////////////////////
  // 障碍物id
  Object.id = "1";
  // 类型
  Object.semantic.type = 1; // CAR=1
  // 形状
  Object.shape.type = 2;
  /*----------------长宽高----------------*/
  Object.shape.dimensions.x = 2.2;
  Object.shape.dimensions.y = 1.6;
  Object.shape.dimensions.z = 0.5;

  // 中心点
  Object.state.pose_covariance.pose.position.x = x_pose_covariance;
  Object.state.pose_covariance.pose.position.y = y_pose_covariance;

  /*----------------顶点----------------*/
  PPoint center(x_pose_covariance, y_pose_covariance);
  oba.CalculateCarBoundaryPoint(Object.shape.dimensions.x, Object.shape.dimensions.y, center, head, left_front, left_buttom,
                                right_buttom, right_front);
  Object.shape.footprint.points.resize(4);
  // 左下角(从侧边看)
  Object.shape.footprint.points[0].x = right_buttom.x;
  Object.shape.footprint.points[0].y = right_buttom.y;
  // 左上角(从侧边看)
  Object.shape.footprint.points[1].x = left_buttom.x;
  Object.shape.footprint.points[1].y = left_buttom.y;
  // 右上角(从侧边看)
  Object.shape.footprint.points[2].x = left_front.x;
  Object.shape.footprint.points[2].y = left_front.y;
  // 右下角(从侧边看)
  Object.shape.footprint.points[3].x = right_front.x;
  Object.shape.footprint.points[3].y = right_front.y;
  // 线速度
  Object.state.twist_covariance.twist.linear.x = 1.0;
  // 朝向
  Object.state.pose_covariance.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, head);
  Objects.objects.push_back(Object);

  /////////////////////////////////////////////////////////////////////////
  // 障碍物id
  // Object.id = "2";
  // // 类型
  // Object.semantic.type = 1; // CAR=1
  // // 形状
  // Object.shape.type = 2;
  // /*----------------长宽高----------------*/
  // Object.shape.dimensions.x = 2.2;
  // Object.shape.dimensions.y = 1.6;
  // Object.shape.dimensions.z = 0.5;

  // // 中心点
  // Object.state.pose_covariance.pose.position.x = x_pose_covariance1;
  // Object.state.pose_covariance.pose.position.y = y_pose_covariance1;

  // /*----------------顶点----------------*/
  // PPoint center1(x_pose_covariance1, y_pose_covariance1);
  // oba.CalculateCarBoundaryPoint(Object.shape.dimensions.x, Object.shape.dimensions.y, center1, head1, left_front, left_buttom,
  //                               right_buttom, right_front);
  // Object.shape.footprint.points.resize(4);
  // // 左下角(从侧边看)
  // Object.shape.footprint.points[0].x = right_buttom.x;
  // Object.shape.footprint.points[0].y = right_buttom.y;
  // // 左上角(从侧边看)
  // Object.shape.footprint.points[1].x = left_buttom.x;
  // Object.shape.footprint.points[1].y = left_buttom.y;
  // // 右上角(从侧边看)
  // Object.shape.footprint.points[2].x = left_front.x;
  // Object.shape.footprint.points[2].y = left_front.y;
  // // 右下角(从侧边看)
  // Object.shape.footprint.points[3].x = right_front.x;
  // Object.shape.footprint.points[3].y = right_front.y;
  // // 线速度
  // Object.state.twist_covariance.twist.linear.x = 1.0;
  // // 朝向
  // Object.state.pose_covariance.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, head1);
  // Objects.objects.push_back(Object);

  Pub_Obstacles.publish(Objects);
}

void ObstacleTest::Publish_StaticObstacle()
{
  object_msgs::DynamicObject Object;
  Objects.objects.clear();
  //////////////////////////////////////////////////////////////////
  // 障碍物id
  Object.id = "0";
  // 类型
  Object.semantic.type = 1; // CAR=1
  // 形状
  Object.shape.type = 2;
  // 顶点
  Object.shape.footprint.points.resize(4);
  // 左下角
  Object.shape.footprint.points[0].x = 28.4072;
  Object.shape.footprint.points[0].y = -35.0751;
  // 左上角
  Object.shape.footprint.points[1].x = 27.7214;
  Object.shape.footprint.points[1].y = -36.6077;
  // 右上角
  Object.shape.footprint.points[2].x = 23.7005;
  Object.shape.footprint.points[2].y = -33.0203;
  // 右下角
  Object.shape.footprint.points[3].x = 25.3506;
  Object.shape.footprint.points[3].y = -32.0688;
  /*----------------长宽高----------------*/
  Object.shape.dimensions.x = sqrt(pow(Object.shape.footprint.points[0].x - Object.shape.footprint.points[1].x, 2) +
                                   pow(Object.shape.footprint.points[0].y - Object.shape.footprint.points[1].y, 2));
  Object.shape.dimensions.y = sqrt(pow(Object.shape.footprint.points[0].x - Object.shape.footprint.points[3].x, 2) +
                                   pow(Object.shape.footprint.points[0].y - Object.shape.footprint.points[3].y, 2));
  Object.shape.dimensions.z = 1;
  // 中心点
  Object.state.pose_covariance.pose.position.x = 26.5318;
  Object.state.pose_covariance.pose.position.y = -34.0422;
  // 线速度
  Object.state.twist_covariance.twist.linear.x = 0;
  // 朝向
  double h = atan2(Object.shape.footprint.points[0].y - Object.shape.footprint.points[3].y,
                   Object.shape.footprint.points[0].x - Object.shape.footprint.points[3].x);
  Object.state.pose_covariance.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, h + M_PI_2);
  Objects.objects.push_back(Object);

  //---------------------混合A*测试障碍物（直接构建一个停车场）--------------------//
  // //////////////////////////////////////////////////////////////////
  // // 障碍物id
  // Object.id = "2";
  // // 类型
  // Object.semantic.type = 1; // CAR=1
  // // 形状
  // Object.shape.type = 2;
  // // 顶点
  // Object.shape.footprint.points.resize(4);
  // // 左下角
  // Object.shape.footprint.points[0].x = 16.321;
  // Object.shape.footprint.points[0].y = 4.23118;
  // // 左上角
  // Object.shape.footprint.points[1].x = 16.1723;
  // Object.shape.footprint.points[1].y = 8.34537;
  // // 右上角
  // Object.shape.footprint.points[2].x = 19.5925;
  // Object.shape.footprint.points[2].y = 8.39493;
  // // 右下角
  // Object.shape.footprint.points[3].x = 19.5429;
  // Object.shape.footprint.points[3].y = 4.33032;

  // /*---------------长宽高----------------*/
  // Object.shape.dimensions.x = 2;
  // Object.shape.dimensions.y = 4;
  // Object.shape.dimensions.z = 0.5;
  // Object.state.pose_covariance.pose.position.x = 17.8081; //
  // Object.state.pose_covariance.pose.position.y = 6.36263; //
  // // 线速度
  // Object.state.twist_covariance.twist.linear.x = 0;
  // // 朝向
  // Object.state.pose_covariance.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0); // head
  // Objects.objects.push_back(Object);
  // /////////////////////////////////////////////////////////////
  // // 障碍物id
  // Object.id = "3";
  // // 类型
  // Object.semantic.type = 1; // CAR=1
  // // 形状
  // Object.shape.type = 2;
  // // 顶点
  // Object.shape.footprint.points.resize(4);
  // // 左下角
  // Object.shape.footprint.points[0].x = 24.008;
  // Object.shape.footprint.points[0].y = 4.2347;
  // // 左上角
  // Object.shape.footprint.points[1].x = 23.8476;
  // Object.shape.footprint.points[1].y = 8.72512;
  // // 右上角
  // Object.shape.footprint.points[2].x = 27.6565;
  // Object.shape.footprint.points[2].y = 8.8454;
  // // 右下角
  // Object.shape.footprint.points[3].x = 27.6565;
  // Object.shape.footprint.points[3].y = 4.35498;

  // /*---------------长宽高----------------*/
  // Object.shape.dimensions.x = 2;
  // Object.shape.dimensions.y = 4;
  // Object.shape.dimensions.z = 0.5;
  // // 中心点  我在静态障碍物中穿插一个动态的障碍物
  // Object.state.pose_covariance.pose.position.x = 25.6919; //  x_pose_covariance
  // Object.state.pose_covariance.pose.position.y = 6.64028; //    y_pose_covariance
  // // 线速度
  // Object.state.twist_covariance.twist.linear.x = 0;
  // // 朝向
  // Object.state.pose_covariance.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0); // head
  // Objects.objects.push_back(Object);
  // ///////////////////////////////////////////////////////////
  // // 障碍物id
  // Object.id = "4";
  // // 类型
  // Object.semantic.type = 1; // CAR=1
  // // 形状
  // Object.shape.type = 2;
  // // 顶点
  // Object.shape.footprint.points.resize(4);
  // // 左下角
  // Object.shape.footprint.points[0].x = 28.2382;
  // Object.shape.footprint.points[0].y = 19.6092;
  // // 左上角
  // Object.shape.footprint.points[1].x = 28.2033;
  // Object.shape.footprint.points[1].y = 20.6311;
  // // 右上角
  // Object.shape.footprint.points[2].x = 37.4278;
  // Object.shape.footprint.points[2].y = 20.7166;
  // // 右下角
  // Object.shape.footprint.points[3].x = 37.4278;
  // Object.shape.footprint.points[3].y = 19.7388;

  // /*---------------长宽高----------------*/
  // Object.shape.dimensions.x = 8;
  // Object.shape.dimensions.y = 1;
  // Object.shape.dimensions.z = 0.5;
  // // 中心点  我在静态障碍物中穿插一个动态的障碍物
  // Object.state.pose_covariance.pose.position.x = 32.7218; //  x_pose_covariance
  // Object.state.pose_covariance.pose.position.y = 20.2277; //    y_pose_covariance
  // // 线速度
  // Object.state.twist_covariance.twist.linear.x = 0;
  // // 朝向
  // Object.state.pose_covariance.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0); // head
  // Objects.objects.push_back(Object);
  ///////////////////////////////////////////////////////////

  Pub_Obstacles.publish(Objects);
}

// 显示障碍物
void ObstacleTest::visualization(object_msgs::DynamicObjectArray Obstacles)
{
  if (Obstacles.objects.size() > 0)
  {
    visualization_msgs::MarkerArray obstacle_MarkerArray;
    for (size_t i = 0; i < Obstacles.objects.size(); i++)
    {
      visualization_msgs::Marker marker;
      Turn_obstacles_into_squares(marker, Obstacles.objects[i], i);
      obstacle_MarkerArray.markers.push_back(marker);
      Obstacle_Visualization.publish(obstacle_MarkerArray);
    }
  }
}

// 已知顶点和中心点，将任何障碍物变成方形的二维平面，计算长和宽,并存储于Marker
void ObstacleTest::Turn_obstacles_into_squares(visualization_msgs::Marker &marker,
                                               const object_msgs::DynamicObject Primitive_obstacle, const int id)
{
  double Length = Primitive_obstacle.shape.dimensions.x;
  double width = Primitive_obstacle.shape.dimensions.y;

  marker.pose.orientation = Primitive_obstacle.state.pose_covariance.pose.orientation;
  marker.header.frame_id = Frame_id;
  marker.header.stamp = ros::Time::now();
  marker.ns = "basic_shapes";
  marker.id = id; // 注意了
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = Primitive_obstacle.state.pose_covariance.pose.position.x;
  marker.pose.position.y = Primitive_obstacle.state.pose_covariance.pose.position.y;
  marker.pose.position.z = 0;
  marker.scale.x = Length;
  marker.scale.y = width;
  marker.scale.z = Primitive_obstacle.shape.dimensions.z;
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0;
  marker.lifetime = ros::Duration();
}

void ObstacleTest::caculate_points(std::string obstacle_path, std::vector<double> &sample_x,
                                   std::vector<double> &sample_y, std::vector<double> &sample_h)
{
  std::ifstream infile;
  double d;
  infile.open(obstacle_path + "/x");
  if (!infile)
    std::cout << "error:" << obstacle_path << std::endl;
  while (infile >> d)
    sample_x.push_back(d);
  infile.close();

  infile.open(obstacle_path + "/y");
  if (!infile)
    std::cout << "error:" << obstacle_path << std::endl;
  while (infile >> d)
    sample_y.push_back(d);
  infile.close();

  for (size_t i = 1; i < sample_y.size(); i++)
  {
    double yaw = atan2(sample_y[i] - sample_y[i - 1], sample_x[i] - sample_x[i - 1]);
    sample_h.push_back(yaw);
  }
}

// 线程函数
void ObstacleTest::thread_obstacle_test(void)
{
  ros::NodeHandle n4;
  ros::Rate loop_rate(10);
  std::string obstacle_test_path = "";
  ros::param::get("obstacle_test_path", obstacle_test_path);
  std::vector<double> sample_x, sample_x1, sample_x2;
  std::vector<double> sample_y, sample_y1, sample_y2;
  std::vector<double> sample_head, sample_head1, sample_head2;

  caculate_points(obstacle_test_path + "/sim_4/", sample_x, sample_y, sample_head);
  caculate_points(obstacle_test_path + "/sim_1/", sample_x1, sample_y1, sample_head1);

  int i = 0, j = 0, k = 0;
  sleep(1); 
  while (n4.ok())
  {
    // 静态障碍物
    Publish_StaticObstacle();

    // 运动障碍物
    // Publish_DynamicObstacle(sample_x1[i], sample_y1[i], sample_head1[i], sample_x[j], sample_y[j], sample_head[j]);

    // sleep(1);
    // j++;
    // i++;
    // if (j >= sample_x.size())
    // {
    //   j = 0;
    // }

    // if (i >= sample_x1.size())
    // {
    //   i = 0;
    // }
    ros::spinOnce();
    loop_rate.sleep();
  }
}

// 显示真实障碍物
void ObstacleTest::thread_obstacle_visualization(void)
{
  ros::NodeHandle n3;
  ros::Rate loop_rate(5);
  while (n3.ok())
  {
    visualization(Objects);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
