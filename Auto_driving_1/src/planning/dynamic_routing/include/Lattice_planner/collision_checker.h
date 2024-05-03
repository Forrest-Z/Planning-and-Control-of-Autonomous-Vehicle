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


#pragma once
#include <memory>
#include <vector>
#include <map>
#include <limits>
#include <string>
#include <utility>
#include <iostream>
#include "reference_point.h"
#include "Obstacle.h"

#include "box2d.h"
#include "FrenetPath.h"
#include "path_time_graph.h"

class CollisionChecker
{
public:
  CollisionChecker();

  CollisionChecker(const std::vector<const Obstacle *> &obstacles, const double ego_vehicle_s, const double ego_vehicle_d,
                   const std::vector<ReferencePoint> &discretized_reference_line,
                   const std::shared_ptr<PathTimeGraph> &ptr_path_time_graph);
  ~CollisionChecker();
  bool InCollision(const DiscretizedTrajectory &discretized_trajectory);

private:
  void BuildPredictedEnvironment(const std::vector<const Obstacle *> &obstacles, const double ego_vehicle_s,
                                 const double ego_vehicle_d,
                                 const std::vector<ReferencePoint> &discretized_reference_line);

  bool IsEgoVehicleInLane(const double ego_vehicle_s, const double ego_vehicle_d);

  bool IsObstacleBehindEgoVehicle(const Obstacle *obstacle, const double ego_vehicle_s,
                                  const std::vector<ReferencePoint> &discretized_reference_line);

private:
  std::vector<std::vector<Box2d>> predicted_bounding_rectangles_;
  std::shared_ptr<PathTimeGraph> ptr_path_time_graph_;
};
