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

#include <string>
#include <vector>

#include "Obstacle.h"
#include "path_data.h"
#include "speed_limit.h"
#include "reference_line.h"
#include "indexed_list.h"

class SpeedLimitDecider
{
public:
    SpeedLimitDecider(const SL_Boundary &adc_sl_boundary,
                      const ReferenceLine &reference_line,
                      const PathData &path_data);

    virtual ~SpeedLimitDecider() = default;

    virtual void GetSpeedLimits(
        const std::vector<const Obstacle *> &obstacles,
        planning::SpeedLimit *const speed_limit_data) const;

private:
    double GetCentricAccLimit(const double kappa) const;

    void GetAvgKappa(const std::vector<PathPoint> &path_points,
                     std::vector<double> *kappa) const;

private:
    const SL_Boundary &adc_sl_boundary_;
    const ReferenceLine &reference_line_;
    const PathData &path_data_;
};
