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
#include "gridded_path_time_graph.h"
#include <algorithm>
#include <future>
#include "point_factory.h"

namespace
{
  static constexpr double kDoubleEpsilon = 1.0e-6;

  // Continuous-time collision check using linear interpolation as closed-loop
  // dynamics
  bool CheckOverlapOnDpStGraph(const std::vector<ST_Boundary> &boundaries, const StGraphPoint &p1, const StGraphPoint &p2)
  {
    if (Config_.FLAGS_use_st_drivable_boundary)
    {
      return false;
    }
    for (const auto &boundary : boundaries)
    {
      if (boundary.boundary_type() == ST_Boundary::BoundaryType::KEEP_CLEAR)
      {
        continue;
      }
      // Check collision between a polygon and a line segment
      if (boundary.HasOverlap({p1.point(), p2.point()}))
      {
        return true;
      }
    }
    return false;
  }
} // namespace

GriddedPathTimeGraph::GriddedPathTimeGraph(const StGraphData &st_graph_data,
                                           const std::vector<const Obstacle *> &obstacles,
                                           const TrajectoryPoint &init_point)
    : st_graph_data_(st_graph_data), init_point_(init_point), dp_st_cost_(st_graph_data_.total_time_by_conf(), st_graph_data_.path_length(), obstacles, init_point_)
{
  total_length_t_ = st_graph_data_.total_time_by_conf();
  unit_t_ = Config_.unit_t;
  total_length_s_ = st_graph_data_.path_length();
  dense_unit_s_ = Config_.dense_unit_s;
  sparse_unit_s_ = Config_.sparse_unit_s;
  dense_dimension_s_ = Config_.dense_dimension_s;

  // std::cout << "dense_dimension_s_:" << dense_dimension_s << "\n";
  // std::cout << "unit_t_:" << unit_t_ << "\n";

  // Safety approach preventing unreachable acceleration/deceleration
  max_acceleration_ = std::min(std::abs(Config_.max_acceleration), std::abs(Config_.max_acceleration));
  max_deceleration_ = -1.0 * std::min(std::abs(Config_.max_deceleration), std::abs(Config_.max_deceleration));
}

/**************************************************************************
 * 速度的主流程：
 * 1、初始化CostTable，   InitCostTable()
 * 2、初始化限速查询表，   InitSpeedLimitLookUp()
 * 3、计算更新CostTable， CalculateTotalCost()
 * 4、回溯，得到SpeedProfile
 * **************************************************************************/
bool GriddedPathTimeGraph::Search(SpeedData *const speed_data)
{
  static constexpr double kBounadryEpsilon = 1e-2;
  //函数Search()的第一部分是一个for循环，遍历的是STGraph中所有的boundary，
  //每一个障碍物都会生成一个对应的boundary，这里就是将所有障碍的boundary都判断一遍

  for (const auto &boundary : st_graph_data_.st_boundaries())
  {
    // std::cout << "upper_points_:"
    //           << boundary.getUpper_points().size() << "\n";
    // std::cout << "Boundary_Type:" << boundary.TypeName() << "\n";
    //KEEP_CLEAR的范围不影响速度规划
    if (boundary.boundary_type() == ST_Boundary::BoundaryType::KEEP_CLEAR)
    {
      continue;
    }
    //若boundary包含原点或非常接近原点（即自车现在的位置），自车应该stop，不再前进
    //故s,v,a都是0，即随着t推移，自车不动
    if (boundary.IsPointInBoundary({0.0, 0.0}) ||
        (std::fabs(boundary.min_t_) < kBounadryEpsilon && std::fabs(boundary.min_s_) < kBounadryEpsilon))
    {
      dimension_t_ = static_cast<uint32_t>(std::ceil(total_length_t_ / static_cast<double>(unit_t_))) + 1;
      std::vector<SpeedPoint> speed_profile;
      double t = 0.0;
      for (uint32_t i = 0; i < dimension_t_; ++i, t += unit_t_)
      {
        speed_profile.push_back(util::PointFactory::ToSpeedPoint(0, t));
      }
      *speed_data = SpeedData(std::move(speed_profile));

      return false;
    }
  }

  if (!InitCostTable())
  {
    const std::string msg = "Initialize cost table failed.";
    std::cout << msg;
    return false;
  }

  if (!InitSpeedLimitLookUp())
  {
    const std::string msg = "Initialize speed limit lookup table failed.";
    std::cout << msg;
    return false;
  }
  if (!CalculateTotalCost())
  {
    const std::string msg = "Calculate total cost failed.";
    std::cout << msg;
    return false;
  }
  if (!RetrieveSpeedProfile(speed_data))
  {
    const std::string msg = "Retrieve best speed profile failed.";
    std::cout << msg;
    return false;
  }
  return true;
}

/*
无人车需要对未来T时间，未来S距离内的时间距离进行规划，最简单的做法就是构建一张表格，例如大小为TxS，行代表0-T时刻的索引；
列代表0-S距离上的索引。其中每个点table[t][s]就可以表示为初始规划点init_point到(相对时刻t，累积距离s)p2点的最小开销cost以及最小开销对应的父节点。
例如现在有两个节点可以连接到table[t][s]，分别为table[t-1][s-1]和table[t-1][s-2]，那么可以分别计算两个节点到当前节点的开销cost1和cost2：
table[t-1][s-2]到table[t][s]的开销为:
                    cost1 = total_cost(t-1,s-2) + edge_cost(t,s,t-1,s-2)
table[t-1][s-1]到table[t][s]的开销为:
                   cost2 = total_cost(t-1,s-1) + edge_cost(t,s,t-1,s-1)
如果cost1小于cost2，那么table[t][s]的总体最小开销total_cost(t,s)就为cost1，且父节点指针指向table[t-1][s-2]。

*/
bool GriddedPathTimeGraph::InitCostTable()
{
  // Time dimension is homogeneous while Spatial dimension has two resolutions,
  // dense and sparse with dense resolution coming first in the spatial horizon

  // Sanity check for numerical stability
  if (unit_t_ < kDoubleEpsilon)
  {
    const std::string msg = "unit_t is smaller than the kDoubleEpsilon.";
    std::cout << msg;
    return false;
  }

  // Sanity check on s dimension setting
  if (dense_dimension_s_ < 1)
  {
    const std::string msg = "dense_dimension_s is at least 1.";
    std::cout << msg << "\n";
    return false;
  }
  // 这一环节主要是把连续的S-T域离散化，，两层for循环完成任务，
  // 外层为T，分辨率为 unit_t_
  // 内层为S，分辨率为 dense_unit_s_ 和 sparse_unit_s_ ，距离近的一段离散的稠密些，距离远的一段稀疏些
  // t方向的离散化，dimension_t_ = total_length_t_ 除以 unit_t_（时间分辨率）。
  dimension_t_ = static_cast<uint32_t>(std::ceil(total_length_t_ / static_cast<double>(unit_t_))) + 1;

  // s方向的离散化（total_length_s_是DP Path中规划出的路径的长度，因此，稀疏采样的长度sparse_length_s就是总的路径长度total_length_s_减去密集采样点的距离）
  double sparse_length_s = total_length_s_ - static_cast<double>(dense_dimension_s_ - 1) * dense_unit_s_;
  // 稀疏采样点个数
  sparse_dimension_s_ = sparse_length_s > std::numeric_limits<double>::epsilon() ? static_cast<uint32_t>(std::ceil(sparse_length_s / sparse_unit_s_)) : 0;
  // 密集采样点个数
  dense_dimension_s_ = sparse_length_s > std::numeric_limits<double>::epsilon() ? dense_dimension_s_ : static_cast<uint32_t>(std::ceil(total_length_s_ / dense_unit_s_)) + 1;
  dimension_s_ = dense_dimension_s_ + sparse_dimension_s_;

  // std::cout << "dense_dimension_s_:" << dense_dimension_s_ << "\n";
  // std::cout << "dimension_t_:" << dimension_t_ << "\n";
  // std::cout << "dimension_s_:" << dimension_s_ << "\n";
  // std::cout << "sparse_length_s:" << sparse_length_s << "\n";

  // Sanity Check
  if (dimension_t_ < 1 || dimension_s_ < 1)
  {
    const std::string msg = "Dp st cost table size incorrect.";
    std::cout << msg;
    return false;
  }
  // cost_table_为双层vector，外层是t，内层是s，
  cost_table_ =
      std::vector<std::vector<StGraphPoint>>(dimension_t_, std::vector<StGraphPoint>(dimension_s_, StGraphPoint()));

  double curr_t = 0.0; // 起点 t = 0
  for (uint32_t i = 0; i < cost_table_.size(); ++i, curr_t += unit_t_)
  {
    auto &cost_table_i = cost_table_[i];
    double curr_s = 0.0;
    for (uint32_t j = 0; j < dense_dimension_s_; ++j, curr_s += dense_unit_s_)
    {
      cost_table_i[j].Init(i, j, STPoint(curr_s, curr_t));
    }
    curr_s = static_cast<double>(dense_dimension_s_ - 1) * dense_unit_s_ + sparse_unit_s_;
    //再对sparse_s 初始化
    for (uint32_t j = dense_dimension_s_; j < cost_table_i.size(); ++j, curr_s += sparse_unit_s_)
    {
      cost_table_i[j].Init(i, j, STPoint(curr_s, curr_t));
    }
  }
  //打印调试，正确
  // std::cout << "[" << cost_table_.size() << "," << dense_dimension_s_ << "]"
  //           << "\n";
  // for (uint32_t i = 0; i < cost_table_.size(); ++i)
  // {
  //   for (uint32_t j = 0; j < cost_table_[0].size(); ++j)
  //   {
  //     std::cout << "s:" << cost_table_[i][j].point().s() << " "
  //               << "t:" << cost_table_[i][j].point().t() << "\n";
  //   }
  // }
  //存储t=0时刻的每个s的值
  const auto &cost_table_0 = cost_table_[0]; //t=0时刻的s

  // std::cout << cost_table_0.size() << "\n"; //252

  spatial_distance_by_index_ = std::vector<double>(cost_table_0.size(), 0.0);
  for (uint32_t i = 0; i < cost_table_0.size(); ++i)
  {
    spatial_distance_by_index_[i] = cost_table_0[i].point().s();
  }
  return true;
}

//t=0的第一列的s的初始速度ds
bool GriddedPathTimeGraph::InitSpeedLimitLookUp()
{
  speed_limit_by_index_.clear();

  speed_limit_by_index_.resize(dimension_s_);
  const auto &speed_limit = st_graph_data_.speed_limit();
  for (uint32_t i = 0; i < dimension_s_; ++i)
  {
    // std::cout << "speed_limit:"
    //           << "s:" << speed_limit.speed_limit_points()[i].first << " "
    //           << "v:" << speed_limit.speed_limit_points()[i].second << "\n";
    speed_limit_by_index_[i] = speed_limit.GetSpeedLimitByS(cost_table_[0][i].point().s());
  }
  return true;
}
/*
这里与S-L域的路径路径规划有点区别，多了些剪枝操作，实现细节见 GetRowRange( )，
依据 max_acceleration_ 和 max_deceleration_ 算出下一时刻可以到达的S， 进而得到范围[next_lowest_row, next_lowest_row]

当前节点的cost计算可以分为4部分：
  障碍物项开销(节点内)
  限速项开销cost(节点间EdgeCost)
  加速度项开销cost(节点间EdgeCost)
  加速度抖动开销cost(节点间EdgeCost)
节点内和节点间区别在于，节点内Cost也就是障碍物项惩罚只用到了当前时刻(t,s)的信息。而计算速度，加速度，加速度抖动需要用到前几个节点的信息，
所以是节点间的cost，这么安排主要增加代码可读性。
*/
bool GriddedPathTimeGraph::CalculateTotalCost()
{
  // col and row are for STGraph
  // t corresponding to col
  // s corresponding to row
  size_t next_highest_row = 0;
  size_t next_lowest_row = 0;
  // 外循环，每一列的index，即每一个t

  for (size_t c = 0; c < cost_table_.size(); ++c)
  {
    size_t highest_row = 0;
    size_t lowest_row = cost_table_.back().size() - 1;
    // count为每一列的点数
    int count = static_cast<int>(next_highest_row) - static_cast<int>(next_lowest_row) + 1;

    if (count > 0)
    {
      std::vector<std::future<void>> results;
      // 内循环，每行的index，即每一个s
      for (size_t r = next_lowest_row; r <= next_highest_row; ++r)
      {
        auto msg = std::make_shared<StGraphMessage>(c, r);
        CalculateCostAt(msg);
      }
    }

    // 下一轮循环的准备工作，更新范围 [next_lowest_row, next_highest_row]
    for (size_t r = next_lowest_row; r <= next_highest_row; ++r)
    {
      const auto &cost_cr = cost_table_[c][r];
      if (cost_cr.total_cost() < std::numeric_limits<double>::infinity())
      {
        size_t h_r = 0;
        size_t l_r = 0;
        GetRowRange(cost_cr, &h_r, &l_r);
        highest_row = std::max(highest_row, h_r);
        lowest_row = std::min(lowest_row, l_r);
      }
    }
    next_highest_row = highest_row;
    next_lowest_row = lowest_row;
  }

  return true;
}

void GriddedPathTimeGraph::GetRowRange(const StGraphPoint &point, size_t *next_highest_row, size_t *next_lowest_row)
{
  double v0 = 0.0;
  // TODO(all): Record speed information in StGraphPoint and deprecate this.
  // A scaling parameter for DP range search due to the lack of accurate
  // information of the current velocity (set to 1 by default since we use
  // past 1 second's average v as approximation)
  double acc_coeff = 0.5;
  if (!point.pre_point())
  {
    v0 = init_point_.v;
  }
  else
  {
    v0 = point.GetOptimalSpeed();
  }

  const auto max_s_size = dimension_s_ - 1;
  const double t_squared = unit_t_ * unit_t_;
  const double s_upper_bound = v0 * unit_t_ + acc_coeff * max_acceleration_ * t_squared + point.point().s();
  const auto next_highest_itr =
      std::lower_bound(spatial_distance_by_index_.begin(), spatial_distance_by_index_.end(), s_upper_bound);
  if (next_highest_itr == spatial_distance_by_index_.end())
  {
    *next_highest_row = max_s_size;
  }
  else
  {
    *next_highest_row = std::distance(spatial_distance_by_index_.begin(), next_highest_itr);
  }

  const double s_lower_bound =
      std::fmax(0.0, v0 * unit_t_ + acc_coeff * max_deceleration_ * t_squared) + point.point().s();
  const auto next_lowest_itr =
      std::lower_bound(spatial_distance_by_index_.begin(), spatial_distance_by_index_.end(), s_lower_bound);
  if (next_lowest_itr == spatial_distance_by_index_.end())
  {
    *next_lowest_row = max_s_size;
  }
  else
  {
    *next_lowest_row = std::distance(spatial_distance_by_index_.begin(), next_lowest_itr);
  }
}

/*
CalculateCostAt( ) 计算每一个点的totalCost，并更新父节点
一个点的total_cost由四部分构成：
1.obstacle_cost：障碍物cost
2.spatial_potential_cost：空间位置cost，于终点s的差值
3.前一个点的total_cost
4.EdgeCost：由三部分构成Speedcost、AccelCost、JerkCost
在更新父节点时同样有一个剪枝操作，使用限速信息planning_upper_speed_limit得到pre_lowest_s，进而将寻找范围限制在[r_low,
r]，其中r为当前行号，因为EMPlanner主要是前进场景，不会考虑倒车情况，那么S值是递增或不变，不会下降，所以r最大也就当前行号



*/
void GriddedPathTimeGraph::CalculateCostAt(const std::shared_ptr<StGraphMessage> &msg)
{
  const uint32_t c = msg->c;
  const uint32_t r = msg->r;
  auto &cost_cr = cost_table_[c][r];

  // 1、计算 obstacle_cost，如果为无穷大，则停止

  cost_cr.SetObstacleCost(dp_st_cost_.GetObstacleCost(cost_cr));

  if (cost_cr.obstacle_cost() > std::numeric_limits<double>::max())
  {
    return;
  }

  // 2、计算SpatialPotentialCost
  cost_cr.SetSpatialPotentialCost(dp_st_cost_.GetSpatialPotentialCost(cost_cr));

  //这里是根据cost_table_的列数（即在采样的时刻的数目）的不同进行分类计算，而不是在判断cost的值。
  //在这里有的博客解析的错误的，请大家注意！！！实际上是分了四类，c==2之后进行的是c>2的计算，只是代码中并没有具体标出来。
  const auto &cost_init = cost_table_[0][0];
  // 第0列的特殊处理，设置起始点的TotalCost 为0。
  if (c == 0)
  {
    // DCHECK_EQ(r, 0) << "Incorrect. Row should be 0 with col = 0. row: " << r;
    cost_cr.SetTotalCost(0.0);
    cost_cr.SetOptimalSpeed(init_point_.v);
    return;
  }

  const double speed_limit = speed_limit_by_index_[r];
  const double cruise_speed = st_graph_data_.cruise_speed();
  // The mininal s to model as constant acceleration formula
  // default: 0.25 * 7 = 1.75 m
  const double min_s_consider_speed = dense_unit_s_ * dimension_t_;
  //第一列的特殊处理
  if (c == 1)
  {
    //当前点的加速度
    const double acc = 2 * (cost_cr.point().s() / unit_t_ - init_point_.v) / unit_t_;
    //加速度、减速度超出范围，返回
    if (acc < max_deceleration_ || acc > max_acceleration_)
    {
      return;
    }

    if (init_point_.v + acc * unit_t_ < -kDoubleEpsilon && cost_cr.point().s() > min_s_consider_speed)
    {
      return;
    }
    // 当前点与起始点的连线与stboundary有重叠，返回
    if (CheckOverlapOnDpStGraph(st_graph_data_.st_boundaries(), cost_cr, cost_init))
    {
      return;
    }
    // 计算当前点的total_cost
    cost_cr.SetTotalCost(cost_cr.obstacle_cost() + cost_cr.spatial_potential_cost() + cost_init.total_cost() +
                         CalculateEdgeCostForSecondCol(r, speed_limit, cruise_speed));
    cost_cr.SetPrePoint(cost_init);
    cost_cr.SetOptimalSpeed(init_point_.v + acc * unit_t_);
    return;
  }

  // 剪枝操作
  static constexpr double kSpeedRangeBuffer = 0.20;
  const double pre_lowest_s = cost_cr.point().s() - Config_.planning_upper_speed_limit * (1 + kSpeedRangeBuffer) * unit_t_;
  // 由当前点推出能到达该点的前一列最小的s
  // 将当前点的pre_col缩小至 [r_low, r]
  const auto pre_lowest_itr =
      std::lower_bound(spatial_distance_by_index_.begin(), spatial_distance_by_index_.end(), pre_lowest_s);
  uint32_t r_low = 0;
  if (pre_lowest_itr == spatial_distance_by_index_.end())
  {
    r_low = dimension_s_ - 1;
  }
  else
  {
    r_low = static_cast<uint32_t>(std::distance(spatial_distance_by_index_.begin(), pre_lowest_itr));
  }
  const uint32_t r_pre_size = r - r_low + 1;
  const auto &pre_col = cost_table_[c - 1];
  double curr_speed_limit = speed_limit;

  // 第二列的特殊处理
  if (c == 2)
  {
    // 对于前一列，遍历从r->r_low的点，
    // 依据重新算得的cost，当前点的pre_point，也就是DP过程的状态转移方程
    for (uint32_t i = 0; i < r_pre_size; ++i)
    {
      uint32_t r_pre = r - i;
      if (std::isinf(pre_col[r_pre].total_cost()) || pre_col[r_pre].pre_point() == nullptr)
      {
        continue;
      }
      // TODO(Jiaxuan): Calculate accurate acceleration by recording speed
      // data in ST point.
      // Use curr_v = (point.s - pre_point.s) / unit_t as current v
      // Use pre_v = (pre_point.s - prepre_point.s) / unit_t as previous v
      // Current acc estimate: curr_a = (curr_v - pre_v) / unit_t
      // = (point.s + prepre_point.s - 2 * pre_point.s) / (unit_t * unit_t)

      //当前点的加速度
      const double curr_a =
          2 * ((cost_cr.point().s() - pre_col[r_pre].point().s()) / unit_t_ - pre_col[r_pre].GetOptimalSpeed()) /
          unit_t_;
      if (curr_a < max_deceleration_ || curr_a > max_acceleration_)
      {
        continue;
      }

      if (pre_col[r_pre].GetOptimalSpeed() + curr_a * unit_t_ < -kDoubleEpsilon &&
          cost_cr.point().s() > min_s_consider_speed)
      {
        continue;
      }

      // Filter out continuous-time node connection which is in collision with
      // obstacle
      if (CheckOverlapOnDpStGraph(st_graph_data_.st_boundaries(), cost_cr, pre_col[r_pre]))
      {
        continue;
      }
      curr_speed_limit = std::fmin(curr_speed_limit, speed_limit_by_index_[r_pre]);
      const double cost = cost_cr.obstacle_cost() + cost_cr.spatial_potential_cost() + pre_col[r_pre].total_cost() +
                          CalculateEdgeCostForThirdCol(r, r_pre, curr_speed_limit, cruise_speed);

      if (cost < cost_cr.total_cost())
      {
        cost_cr.SetTotalCost(cost);
        cost_cr.SetPrePoint(pre_col[r_pre]); //找到每一个当前点的pre_point
        cost_cr.SetOptimalSpeed(pre_col[r_pre].GetOptimalSpeed() + curr_a * unit_t_);
      }
    }
    return;
  }

  // 第3列，及以后列的处理
  // 依据重新算得的cost，当前点的pre_point，也就是DP过程的状态转移方程
  for (uint32_t i = 0; i < r_pre_size; ++i)
  {
    uint32_t r_pre = r - i;
    if (std::isinf(pre_col[r_pre].total_cost()) || pre_col[r_pre].pre_point() == nullptr)
    {
      continue;
    }
    // Use curr_v = (point.s - pre_point.s) / unit_t as current v
    // Use pre_v = (pre_point.s - prepre_point.s) / unit_t as previous v
    // Current acc estimate: curr_a = (curr_v - pre_v) / unit_t
    // = (point.s + prepre_point.s - 2 * pre_point.s) / (unit_t * unit_t)
    const double curr_a =
        2 * ((cost_cr.point().s() - pre_col[r_pre].point().s()) / unit_t_ - pre_col[r_pre].GetOptimalSpeed()) / unit_t_;
    if (curr_a > max_acceleration_ || curr_a < max_deceleration_)
    {
      continue;
    }

    if (pre_col[r_pre].GetOptimalSpeed() + curr_a * unit_t_ < -kDoubleEpsilon &&
        cost_cr.point().s() > min_s_consider_speed)
    {
      continue;
    }

    if (CheckOverlapOnDpStGraph(st_graph_data_.st_boundaries(), cost_cr, pre_col[r_pre]))
    {
      continue;
    }

    uint32_t r_prepre = pre_col[r_pre].pre_point()->index_s();
    const StGraphPoint &prepre_graph_point = cost_table_[c - 2][r_prepre];
    if (std::isinf(prepre_graph_point.total_cost()))
    {
      continue;
    }

    if (!prepre_graph_point.pre_point())
    {
      continue;
    }
    const STPoint &triple_pre_point = prepre_graph_point.pre_point()->point();
    const STPoint &prepre_point = prepre_graph_point.point();
    const STPoint &pre_point = pre_col[r_pre].point();
    const STPoint &curr_point = cost_cr.point();
    curr_speed_limit = std::fmin(curr_speed_limit, speed_limit_by_index_[r_pre]);
    double cost =
        cost_cr.obstacle_cost() + cost_cr.spatial_potential_cost() + pre_col[r_pre].total_cost() +
        CalculateEdgeCost(triple_pre_point, prepre_point, pre_point, curr_point, curr_speed_limit, cruise_speed);

    if (cost < cost_cr.total_cost())
    {
      cost_cr.SetTotalCost(cost);
      cost_cr.SetPrePoint(pre_col[r_pre]);
      cost_cr.SetOptimalSpeed(pre_col[r_pre].GetOptimalSpeed() + curr_a * unit_t_);
    }
  }
}
/*
RetrieveSpeedProfile( ) 顺藤摸瓜，找到最优S-T曲线
需要注意回溯起点的选择
遍历每一列的最后一个点，找正在的best_end_point，并更新min_cost
这里不直接使用最后一列的min_cost点作为终点
因为采样时间是一个预估时间窗口，在这之前可能就到达终点了
*/
bool GriddedPathTimeGraph::RetrieveSpeedProfile(SpeedData *const speed_data)
{
  double min_cost = std::numeric_limits<double>::infinity();
  const StGraphPoint *best_end_point = nullptr;
  // 从cost_table_最后一列找 min_cost
  for (const StGraphPoint &cur_point : cost_table_.back())
  {
    if (!std::isinf(cur_point.total_cost()) && cur_point.total_cost() < min_cost)
    {
      best_end_point = &cur_point;
      min_cost = cur_point.total_cost();
    }
  }
  // 遍历每一列的最后一个点，找正在的best_end_point，并更新min_cost
  // 这里不直接使用最后一列的min_cost点作为终点
  // 因为采样时间是一个预估时间窗口，在这之前可能就到达终点了
  for (const auto &row : cost_table_)
  {
    const StGraphPoint &cur_point = row.back();
    if (!std::isinf(cur_point.total_cost()) && cur_point.total_cost() < min_cost)
    {
      best_end_point = &cur_point;
      min_cost = cur_point.total_cost();
    }
  }

  if (best_end_point == nullptr)
  {
    const std::string msg = "Fail to find the best feasible trajectory.";
    // std::cout << msg;
    return false;
  }
  //打印最优的速度终点
  // std::cout << "s:" << best_end_point->point().s() << " "
  //           << "t:" << best_end_point->point().t()
  //           << "\n";

  // 回溯，得到最优的 speed_profile
  std::vector<SpeedPoint> speed_profile;
  const StGraphPoint *cur_point = best_end_point;
  while (cur_point != nullptr)
  {
    // std::cout << "Time: " << cur_point->point().t();
    // std::cout << "S: " << cur_point->point().s();
    // std::cout << "V: " << cur_point->GetOptimalSpeed();
    SpeedPoint speed_point;
    speed_point.set_s(cur_point->point().s());
    speed_point.set_t(cur_point->point().t());
    speed_profile.push_back(speed_point);
    cur_point = cur_point->pre_point();
  }
  std::reverse(speed_profile.begin(), speed_profile.end()); //找到从起始点到达最佳终点的所有点组成的序列
  // std::cout << "---------------original-------------------"
  //           << "\n";

  // for (size_t i = 0; i < speed_profile.size(); i++)
  // {
  //   std::cout << "s:" << speed_profile[i].s << " "
  //             << "t:" << speed_profile[i].t
  //             << "\n";
  // }

  static constexpr double kEpsilon = std::numeric_limits<double>::epsilon();
  if (speed_profile.front().t > kEpsilon || speed_profile.front().s > kEpsilon)
  {
    const std::string msg = "Fail to retrieve speed profile.";
    std::cout << msg;
    return false;
  }

  // 计算每个点的速度 v
  for (size_t i = 0; i + 1 < speed_profile.size(); ++i)
  {
    const double v =
        (speed_profile[i + 1].s - speed_profile[i].s) / (speed_profile[i + 1].t - speed_profile[i].t + 1e-3);
    speed_profile[i].set_v(v);
  }

  *speed_data = SpeedData(std::move(speed_profile));
  // std::cout << "speed_data:" << speed_data->size() << "\n";
  // std::cout << "---------------original-------------------"
  //           << "\n";
  // for (size_t i = 0; i < speed_data->size(); i++)
  // {
  //   std::cout << "(" << speed_data->at(i).t << "," << speed_data->at(i).v << ")"
  //             << "\n";
  // }
  return true;
}

double GriddedPathTimeGraph::CalculateEdgeCost(const STPoint &first, const STPoint &second, const STPoint &third,
                                               const STPoint &forth, const double speed_limit,
                                               const double cruise_speed)
{
  return dp_st_cost_.GetSpeedCost(third, forth, speed_limit, cruise_speed) +
         dp_st_cost_.GetAccelCostByThreePoints(second, third, forth) +
         dp_st_cost_.GetJerkCostByFourPoints(first, second, third, forth);
}

double GriddedPathTimeGraph::CalculateEdgeCostForSecondCol(const uint32_t row, const double speed_limit,
                                                           const double cruise_speed)
{
  double init_speed = init_point_.v;
  double init_acc = init_point_.a;
  const STPoint &pre_point = cost_table_[0][0].point();
  const STPoint &curr_point = cost_table_[1][row].point();
  return dp_st_cost_.GetSpeedCost(pre_point, curr_point, speed_limit, cruise_speed) +
         dp_st_cost_.GetAccelCostByTwoPoints(init_speed, pre_point, curr_point) +
         dp_st_cost_.GetJerkCostByTwoPoints(init_speed, init_acc, pre_point, curr_point);
}

double GriddedPathTimeGraph::CalculateEdgeCostForThirdCol(const uint32_t curr_row, const uint32_t pre_row,
                                                          const double speed_limit, const double cruise_speed)
{
  double init_speed = init_point_.v;
  const STPoint &first = cost_table_[0][0].point();
  const STPoint &second = cost_table_[1][pre_row].point();
  const STPoint &third = cost_table_[2][curr_row].point();
  return dp_st_cost_.GetSpeedCost(second, third, speed_limit, cruise_speed) +
         dp_st_cost_.GetAccelCostByThreePoints(first, second, third) +
         dp_st_cost_.GetJerkCostByThreePoints(init_speed, first, second, third);
}
