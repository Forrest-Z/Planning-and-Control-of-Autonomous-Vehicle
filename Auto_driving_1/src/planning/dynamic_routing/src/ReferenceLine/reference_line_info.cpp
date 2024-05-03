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
#include "reference_line_info.h"

ReferenceLineInfo::ReferenceLineInfo(const InitialConditions &adc_planning_point, const ReferenceLine &reference_line) : reference_line_(reference_line)
{
}

const std::vector<PathBoundary> &ReferenceLineInfo::GetCandidatePathBoundaries() const
{
  return candidate_path_boundaries_;
}

void ReferenceLineInfo::SetCandidatePathBoundaries(std::vector<PathBoundary> &&path_boundaries)
{
  candidate_path_boundaries_ = std::move(path_boundaries);
}
const SpeedData &ReferenceLineInfo::speed_data() const
{
  return speed_data_;
}

const ReferenceLine &ReferenceLineInfo::reference_line() const
{
  return reference_line_;
}

void ReferenceLineInfo::set_speed_data(SpeedData &speed_)
{
  speed_data_ = speed_;
}