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
#include "path_time_heuristic_optimizer.h"
/*

离散化连续的S-T域，这里和路径规划的S-L域有点小区别，S轴方法包含稠密区dense_dimension_s_和稀疏区sparse_dimension_s_，
cost_table_。这里用的是vector<vector<>>，路径规划用的是list<list<>>
forward过程，递推出每个节点的cost，这里有一个剪枝操作，依据max_acceleration_和max_deceleration_找到下一列可能到达的范围[next_lowest_row,
next_highest_row]
cost和父节点更新，这里同样有个剪枝操作，依据 planning_upper_speed_limit得到pre_lowest_s，最后得到追溯范围 [r_low,
r] backward过程，顺藤摸瓜，得到最优的一串ST point


PathTimeHeuristicOptimizer:是速度DP规划的入口
GriddedPathTimeGraph:完成具体的动态规划DP过程
StGraphPoint定义了每个采样点数据形式，
DpStCost进行每个点的cost计算

*/
PathTimeHeuristicOptimizer::PathTimeHeuristicOptimizer(const StGraphData &st_graph_data,
                                                       const std::vector<const Obstacle *> &obstacles)
    : st_graph_data_(st_graph_data), obstacles_(obstacles)
{
}

bool PathTimeHeuristicOptimizer::Process(const PathData &path_data, const TrajectoryPoint &init_point,
                                         SpeedData *const speed_data)
{
  init_point_ = init_point;
  if (path_data.discretized_path().empty()) //如果路径规划没有路径
  {
    // std::cout << "Empty path data";
    return false;
  }

  if (!SearchPathTimeGraph(speed_data))
  {
    // AERROR << "Failed to search graph with dynamic programming.";
    return false;
  }
  return true;
}

//调用SearchPathTimeGraph( )完成规划任务，最终的DP过程在Search()完成
bool PathTimeHeuristicOptimizer::SearchPathTimeGraph(SpeedData *speed_data) const
{
  GriddedPathTimeGraph st_graph(st_graph_data_, obstacles_, init_point_);

  if (!st_graph.Search(speed_data))
  {
    // std::cout << "failed to search graph with dynamic programming.";
    return false;
  }
  return true;
}
