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
  Modification: Only some functions are referenced
  Modification: Modify the input of the constructor
**/

#ifndef ACCELERATIONTRAJECTORY1D_H
#define ACCELERATIONTRAJECTORY1D_H
#include "path_struct.h"
#include <vector>
#include <cmath>
#include <queue>
#include <string>
class PiecewiseAccelerationTrajectory1d : public Curve1d
{
public:
  PiecewiseAccelerationTrajectory1d(const double start_s, const double start_v);

  virtual ~PiecewiseAccelerationTrajectory1d() = default;

  void AppendSegment(const double a, const double t_duration);

  void PopSegment();

  double ParamLength() const override;

  double Evaluate(const std::uint32_t order, const double param) const override;

  std::string ToString() const override;

  std::array<double, 4> Evaluate(const double t) const;

private:
  double Evaluate_s(const double t) const;

  double Evaluate_v(const double t) const;

  double Evaluate_a(const double t) const;

  double Evaluate_j(const double t) const;

private:
  // accumulated s
  std::vector<double> s_;

  std::vector<double> v_;

  // accumulated t
  std::vector<double> t_;

  std::vector<double> a_;
};
#endif