/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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
 * Modified function input and used only some functions
 **/
#pragma once
#include "path_points.h"
#include <iostream>
#include <algorithm>
#include <mutex>
#include <utility>

class SpeedData : public std::vector<SpeedPoint>
{
public:
  SpeedData() = default;

  explicit SpeedData(std::vector<SpeedPoint> speed_points);

  void AppendSpeedPoint(const double s, const double time, const double v, const double a, const double da);

  bool EvaluateByTime(const double time, SpeedPoint *const speed_point) const;

  // Assuming spatial traversed distance is monotonous, which is the case for
  // current usage on city driving scenario
  bool EvaluateByS(const double s, SpeedPoint *const speed_point) const;

  double TotalTime() const;

  // Assuming spatial traversed distance is monotonous
  double TotalLength() const;

  bool Empty() const;

  virtual std::string DebugString() const;
};