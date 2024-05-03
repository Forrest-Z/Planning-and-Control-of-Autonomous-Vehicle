#ifndef REFERENCELINE_SPLIT_H
#define REFERENCELINE_SPLIT_H

#include <nav_msgs/Path.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <fstream>
#include <string>
#include <ros/ros.h>
#include "global_path_struct.h"
#include "cos_theta_smoother.h"
#include "fem_pos_deviation_smoother.h"

class ReferenceLine_Split
{
public:
  ReferenceLine_Split();
  ~ReferenceLine_Split();
  void referenceLine_split(Eigen::MatrixXd &hdmap_way_points);
  void Publish(Eigen::MatrixXd &path_point_after_interpolation_);
  void NormalizePoints(std::vector<std::pair<double, double>> *xy_points);
  void DeNormalizePoints(std::vector<std::pair<double, double>> *xy_points);
  void average_interpolation(Eigen::MatrixXd &input, Eigen::MatrixXd &output, double interval_dis,
                             double distance);

private:
  Eigen::MatrixXd path_point_after_interpolation;
  ros::Publisher referenceline_pub_;
  nav_msgs::Path referenceline;
  Eigen::MatrixXd way_point, rest_point;
  bool which_smoothers; // 选择参考线平滑方式
  double zero_x_ = 0.0;
  double zero_y_ = 0.0;
};

#endif // REFERENCELINE_SPLIT_H