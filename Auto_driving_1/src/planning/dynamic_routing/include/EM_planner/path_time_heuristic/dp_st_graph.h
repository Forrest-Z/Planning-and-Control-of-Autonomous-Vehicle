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
  Modification: Only some functions are referenced
**/
#pragma once
#include <memory>
#include "speed_data.h"
#include "st_graph_data.h"
#include "Obstacle.h"
#include "dp_st_cost.h"
#include "st_graph_point.h"

class DpStGraph
{
public:
    DpStGraph(const StGraphData &st_graph_data,
              const std::vector<const Obstacle *> &obstacles,
              const TrajectoryPoint &init_point,
              const SL_Boundary &adc_sl_boundary);

    bool Search(SpeedData *const speed_data);

private:
    bool InitCostTable();

    bool RetrieveSpeedProfile(SpeedData *const speed_data);

    bool CalculateTotalCost();

    // defined for cyber task
    struct StGraphMessage
    {
        StGraphMessage(const uint32_t c_, const int32_t r_) : c(c_), r(r_) {}
        uint32_t c;
        uint32_t r;
    };
    void CalculateCostAt(const std::shared_ptr<StGraphMessage> &msg);

    double CalculateEdgeCost(const STPoint &first, const STPoint &second,
                             const STPoint &third, const STPoint &forth,
                             const double speed_limit);
    double CalculateEdgeCostForSecondCol(const uint32_t row,
                                         const double speed_limit);
    double CalculateEdgeCostForThirdCol(const uint32_t curr_r,
                                        const uint32_t pre_r,
                                        const double speed_limit);

    void GetRowRange(const StGraphPoint &point, size_t *highest_row,
                     size_t *lowest_row);

private:
    const StGraphData &st_graph_data_;

    // obstacles based on the current reference line
    const std::vector<const Obstacle *> &obstacles_;

    // initial status
    TrajectoryPoint init_point_;

    // cost utility with configuration;
    DpStCost dp_st_cost_;

    const SL_Boundary &adc_sl_boundary_;

    double unit_s_ = 0.0;
    double unit_t_ = 0.0;

    // cost_table_[t][s]
    // row: s, col: t --- NOTICE: Please do NOT change.
    std::vector<std::vector<StGraphPoint>> cost_table_;
};