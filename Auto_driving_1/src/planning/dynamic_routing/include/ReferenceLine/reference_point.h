/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file
  Modification: used only some functions
**/
#pragma once
#include <string>
#include <vector>
#include <iostream>
#include <array>
#include "vec2d.h"

//参考线上的点的类
class ReferencePoint
{
public:
  ReferencePoint() = default;
  ~ReferencePoint() = default;
  ReferencePoint(const double kappa_, const double dkappa_, const double x_, const double y_, const double heading_,
                 const double accumulated_s_);
  void cartesian_to_frenet(const double rs, const double rx, const double ry, const double rtheta, const double x,
                           const double y, double* ptr_s, double* ptr_d);
  void frenet_to_cartesian(const double rs, const double rx, const double ry, const double rtheta, const double rkappa,
                           const double rdkappa, const std::array<double, 3>& s_condition,
                           const std::array<double, 3>& d_condition, double* const ptr_x, double* const ptr_y,
                           double* const ptr_theta, double* const ptr_kappa, double* const ptr_v, double* const ptr_a);
  double kappa() const;
  double dkappa() const;
  double heading() const;
  double s() const;
  double get_x() const;
  double get_y() const;

  void set_x(double x);
  void set_y(double y);
  void set_heading(double heading);
  void set_kappa(double kappa);
  void set_dkappa(double dkappa);
  void set_s(double s);

  double kappa_;
  double dkappa_;
  double x_;              // x position
  double y_;              // y position
  double heading_;        // yaw in rad
  double accumulated_s_;  // s
};
