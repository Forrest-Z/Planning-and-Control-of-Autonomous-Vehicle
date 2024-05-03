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
**/
#pragma once
#include <utility>
#include <vector>

#include "cos_theta_ipopt_interface.h"

class CosThetaSmoother
{
public:
  explicit CosThetaSmoother();

  bool Solve(const std::vector<std::pair<double, double>>& raw_point2d, const std::vector<double>& bounds,
             std::vector<double>* opt_x, std::vector<double>* opt_y);

private:
};
