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
  Modification: Modify the input of the constructor
**/
#pragma once

#include <utility>
#include <vector>
#include <osqp/osqp.h>
#include <osqp/glob_opts.h>
#include <osqp/cs.h>
#include <osqp/auxil.h>
#include <osqp/scaling.h>
#include <ros/ros.h>
#include "path_struct.h"
#include "piecewise_jerk_trajectory1d.h"
#include "lateral_qp_optimizer.h"

class LateralOSQPOptimizer : public LateralQPOptimizer
{
public:
  LateralOSQPOptimizer() = default;

  virtual ~LateralOSQPOptimizer() = default;

  bool optimize(const std::array<double, 3> &d_state, const double delta_s,
                const std::vector<std::pair<double, double>> &d_bounds) override;

private:
  void CalculateKernel(const std::vector<std::pair<double, double>> &d_bounds, std::vector<c_float> *P_data,
                       std::vector<c_int> *P_indices, std::vector<c_int> *P_indptr);
  void CHECK_EQ(int size1, int size2);
  double delta_s_ = 0.0;
  ;
};