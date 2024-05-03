#ifndef OBSTACLE_TEST_H
#define OBSTACLE_TEST_H
#include <ros/ros.h>
#include <string>
#include <vector>
#include <fstream>
#include <eigen3/Eigen/Dense>
#include <boost/thread.hpp>
#include "Obstacle.h"
#include "object_msgs/DynamicObjectArray.h"
#include "object_msgs/Semantic.h"
#include "object_msgs/Shape.h"
#include "object_msgs/State.h"
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include "ObjectDecision.h"

using namespace Eigen;

class ObstacleTest
{
public:
  ObstacleTest();
  ~ObstacleTest();
  void Publish_StaticObstacle();

  void Publish_DynamicObstacle(double x_pose_covariance, double y_pose_covariance, double head,
                               double x_pose_covariance1, double y_pose_covariance1, double head1);
  void thread_obstacle_test(void);
  void thread_obstacle_visualization(void);

  void caculate_points(std::string obstacle_path, std::vector<double> &sample_x,
                       std::vector<double> &sample_y, std::vector<double> &sample_h);
  void Turn_obstacles_into_squares(visualization_msgs::Marker &marker,
                                   const object_msgs::DynamicObject Primitive_obstacle, const int id);
  void visualization(object_msgs::DynamicObjectArray Obstacles);

private:
  // thread
  boost::thread *routing_thread_obstacle_test;
  boost::thread *routing_thread_obstacle;
  ros::Publisher Pub_Obstacles;
  ros::Publisher Obstacle_Visualization;
  object_msgs::DynamicObjectArray Objects;
  Obstacle_avoid oba;
};

#endif // OBSTACLE_TEST_H
