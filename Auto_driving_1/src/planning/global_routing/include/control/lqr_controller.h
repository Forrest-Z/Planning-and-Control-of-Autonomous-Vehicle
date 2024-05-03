#pragma once

#define EPS 1.0e-4
#define PI 3.1415926

#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include <cmath>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

using namespace std;
using namespace Eigen;

class LqrController
{
public:
  LqrController() = default;
  ~LqrController() = default;

  MatrixXd calRicatti(MatrixXd A, MatrixXd B, MatrixXd Q, MatrixXd R);
  double normalizeAngle(double angle);

  void lqr_control(const nav_msgs::Odometry &robot_state,const nav_msgs::Path &trj_point,
                   const vector<double> trj_k, const vector<double> trj_t, const vector<double> trj_v,
                   double &out_turn_agl, int &out_index);
};


