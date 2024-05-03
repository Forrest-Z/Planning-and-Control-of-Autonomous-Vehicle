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

#include "trajectory_cost.h"
#include "math_utils.h"
#include "point_factory.h"

TrajectoryCost::TrajectoryCost(const VehicleConfig vehicle_config, const ReferenceLine &reference_line,
                               const bool is_change_lane_path, const std::vector<const Obstacle *> &obstacles,
                               const SpeedData &heuristic_speed_data, const SLPoint &init_sl_point)
    : vehicle_config_(vehicle_config),             //自主车的信息
      reference_line_(&reference_line),            //参考线
      is_change_lane_path_(is_change_lane_path),   //改变车道
      heuristic_speed_data_(heuristic_speed_data), //启发速度
      init_sl_point_(init_sl_point)                //初始点的(s,l)

{
  // FLAGS_prediction_total_time其实就是PRediction模块中障碍物轨迹预测总时间，5s。
  // eval_time_interval其实就是Prediction模块中障碍物两个预测轨迹点的时间差，0.1s。
  //所以一共差不多50个位置点。最后对参考线上的每个障碍物在每个时间点设定位置标定框。
  const double total_time = std::min(heuristic_speed_data_.TotalTime(), Config_.FLAGS_prediction_total_time);
  num_of_time_stamps_ = static_cast<uint32_t>(std::floor(total_time / Config_.eval_time_interval));

  // 遍历sl图的障碍物
  /*
  对于每个障碍物，进行如下条件判断：
    情况1：如果是无人车可忽略的障碍物或者迫使无人车停车的障碍物，就不需要考虑。因为前者对无人车前进无影响；后者情况无人车智能停车，根本不需要前进了。
    情况2：如果无人车和某个时间点的障碍物横向距离比较大，那么无人车可以毫无影响的前进，所以该类障碍物也可以忽略。
    情况3：如果障碍物是虚拟障碍物，那么无人车可以毫无影响的前进，所以该类障碍物也可以忽略。
    情况4：如果障碍物是静止的，那么将其加入静止障碍物队列
    情况5：如果障碍物时运动的，那么就需要计算障碍物在该时间点的位置，并将其加入动态障碍物队列
  */

  for (const auto *ptr_obstacle : obstacles)
  {
    const auto &sl_boundary = ptr_obstacle->PerceptionSLBoundary();

    const double adc_left_l = init_sl_point_.l + Config_.left_edge_to_center;
    const double adc_right_l = init_sl_point_.l - Config_.right_edge_to_center;

    //调试参数，正确
    // std::cout << "start_l_:" << sl_boundary.start_l_ << std::endl;
    // std::cout << "end_l_:" << sl_boundary.end_l_ << std::endl;
    // std::cout << "IsStatic：" << ptr_obstacle->IsStatic() << std::endl;
    // std::cout << "x:" << ptr_obstacle->centerpoint.position.x << std::endl;
    // std::cout << "obstacle_length:" << ptr_obstacle->obstacle_length << std::endl;
    // std::cout << "type:" << ptr_obstacle->decision_.nudge_.TypeName() << std::endl;

    if (adc_left_l + Config_.FLAGS_lateral_ignore_buffer < sl_boundary.start_l_ ||
        adc_right_l - Config_.FLAGS_lateral_ignore_buffer > sl_boundary.end_l_)
    {
      continue;
    }

    //这里就假设不是car的话，就是bycycle_or_pedestrian
    bool is_bycycle_or_pedestrian = (sl_boundary.obstacle_type != 1);

    if (ptr_obstacle->IsVirtual())
    {
      // Virtual obstacle
      continue;
    }
    else if (ptr_obstacle->IsStatic() || is_bycycle_or_pedestrian) //静态障碍物或者速度低的
    {
      static_obstacle_sl_boundaries_.push_back(std::move(sl_boundary));
    }
    else //动态障碍物，先不考虑，因为会涉及障碍物的预测轨迹,还没写，我们先从简单的场景开始学
    {
      // 计算每个时间点的位置，转换成标定框加入动态障碍物队列
      std::vector<Box2d> box_by_time;
      for (uint32_t t = 0; t <= num_of_time_stamps_; ++t)
      {
        TrajectoryPoint trajectory_point = ptr_obstacle->GetPointAtTime(t * Config_.eval_time_interval);

        Box2d obstacle_box = ptr_obstacle->GetBoundingBox(trajectory_point);
        static constexpr double kBuff = 0.5;
        Box2d expanded_obstacle_box = Box2d(obstacle_box.center(), obstacle_box.heading(),
                                            obstacle_box.length() + kBuff, obstacle_box.width() + kBuff);
        box_by_time.push_back(expanded_obstacle_box);
      }
      dynamic_obstacle_boxes_.push_back(std::move(box_by_time));
    }
  }
}

// node间的cost主要分为3部分：路径平滑、避开静态障碍物、避开动态障碍物，使用ComparableCost类描述。
ComparableCost TrajectoryCost::Calculate(const QuinticPolynomialCurve1d &curve, const double start_s,
                                         const double end_s, const uint32_t curr_level, const uint32_t total_level)
{
  ComparableCost total_cost;
  // path cost
  total_cost += CalculatePathCost(curve, start_s, end_s, curr_level, total_level);
  // std::cout << "CalculatePathCost:" << total_cost.smoothness_cost << "\n";
  // static obstacle cost
  total_cost += CalculateStaticObstacleCost(curve, start_s, end_s);
  // std::cout << "CalculateStaticObstacleCost:" << total_cost.smoothness_cost << "\n";

  // dynamic obstacle cost
  total_cost += CalculateDynamicObstacleCost(curve, start_s, end_s);
  // std::cout << "CalculateStaticObstacleCost:" << total_cost.smoothness_cost << "\n";

  return total_cost;
}

//指的是两个采样点之间边的cost，以path_resolution为分辨率对这段曲线离散化，一小节一小节的计算，损失函数为 [公式]
//，主要是平滑性的考量，再加上一个末态惩罚。
ComparableCost TrajectoryCost::CalculatePathCost(const QuinticPolynomialCurve1d &curve, const double start_s,
                                                 const double end_s, const uint32_t curr_level,
                                                 const uint32_t total_level)
{
  ComparableCost cost;
  double path_cost = 0.0;
  std::function<double(const double)> quasi_softmax = [this](const double x)
  {
    const double l0 = Config_.path_l_cost_param_l0;
    const double b = Config_.path_l_cost_param_b;
    const double k = Config_.path_l_cost_param_k;
    return (b + std::exp(-k * (x - l0))) / (1.0 + std::exp(-k * (x - l0)));
  };
  // std::cout << "end_s - start_s:" << end_s - start_s << "\n";
  for (double curve_s = 0.0; curve_s < (end_s - start_s); curve_s += Config_.path_resolution)
  {
    const double l = curve.Evaluate(0, curve_s);
    // 0阶导的cost
    path_cost += l * l * Config_.path_l_cost * quasi_softmax(std::fabs(l));

    //一阶导的cost
    const double dl = std::fabs(curve.Evaluate(1, curve_s));
    if (IsOffRoad(curve_s + start_s, l, dl, is_change_lane_path_)) //是否在车道内
    {
      // std::cout << "IsOffRoad"
      //              "\n";
      cost.cost_items[ComparableCost::OUT_OF_BOUNDARY] = true;
    }

    path_cost += dl * dl * Config_.path_dl_cost;
    //二阶导的cost
    const double ddl = std::fabs(curve.Evaluate(2, curve_s));
    path_cost += ddl * ddl * Config_.path_ddl_cost;
  }
  path_cost *= Config_.path_resolution;
  // std::cout << "curr_level:" << curr_level << " "
  //           << "total_level:" << total_level << "\n";
  // 最后一层，距离车道中心线的cost,奇怪了,其实不会执行这个,但是Apollo这么写
  if (curr_level == total_level - 1)
  {
    const double end_l = curve.Evaluate(0, end_s - start_s);
    // std::cout << "aend_s - start_s:" << end_s - start_s << "\n";
    path_cost += std::sqrt(end_l - init_sl_point_.l / 2.0) * Config_.path_end_l_cost;
  }

  cost.smoothness_cost = path_cost;
  return cost;
}

/*
静态障碍物开销其实思路和路径开销一样，在这条多项式曲线上采样，计算每个采样点和所有障碍物的标定框是否重叠，重叠cost就比较大。分情况：
  情况1：障碍物和无人车侧方向上的距离大于一个阈值(lateral_ignore_buffer，默认3m)，那么cost就是0，忽略障碍物
  情况2：如果障碍物在无人车后方，cost为0，可忽略。否则就计算cost
*/
ComparableCost TrajectoryCost::CalculateStaticObstacleCost(const QuinticPolynomialCurve1d &curve, const double start_s,
                                                           const double end_s)
{
  ComparableCost obstacle_cost;
  if (static_obstacle_sl_boundaries_.empty())
  {
    return obstacle_cost;
  }
  //从ego的 start_s 到ens_s计算 所有静态障碍物的cost
  for (double curr_s = start_s; curr_s <= end_s; curr_s += Config_.path_resolution)
  {
    const double curr_l = curve.Evaluate(0, curr_s - start_s);
    for (const auto &obs_sl_boundary : static_obstacle_sl_boundaries_)
    {
      //具体的cost计算交给GetCostFromObsSL()来完成，距离小于 0.6m
      //才计算cost，这里没用常见的距离平方倒数作为惩罚项，而是用Sigmoid函数，这样可以避免距离很小时cost陡增，其他惩罚项都被淹没了
      obstacle_cost += GetCostFromObsSL(curr_s, curr_l, obs_sl_boundary);
    }
  }
  obstacle_cost.safety_cost *= Config_.path_resolution;
  return obstacle_cost;
}

/*
根据预测出来的动态障碍物的轨迹，也是用离散的方式，一小节一小节的判断，当时间接近、位置接近时，借助GetCostBetweenObsBoxes()计算cost
两层for循环完成任务
外循环，依据time_stamp，拿到自车的信息，即ego_box
内循环，ego_box依次与所有的 dynamic_obstacle_boxes_ 中时刻相同的box计算cost

区别静态障碍物，动态障碍物会在这个时间段内运动，在前面我们已经对障碍物进行时间段运动采样，得到了障碍物每隔0.1s的坐标位置，
那么只要需要计算每个采样点和这些运动的坐标位置cost，求和就可以每个动态障碍物的cost.
*/
ComparableCost TrajectoryCost::CalculateDynamicObstacleCost(const QuinticPolynomialCurve1d &curve, const double start_s,
                                                            const double end_s) const
{
  ComparableCost obstacle_cost;
  if (dynamic_obstacle_boxes_.empty())
  {
    return obstacle_cost;
  }
  // for形成的两层循环
  // 外循环，依据time_stamp，拿到自车的信息，即ego_box
  // 内循环，ego_box依次与所有的 dynamic_obstacle_boxes_ 计算cost
  double time_stamp = 0.0;
  // 障碍物每隔eval_time_interval(默认0.1s)会得到一个运动坐标，分别计算对所有时间点坐标的cost
  for (size_t index = 0; index < num_of_time_stamps_; ++index, time_stamp += Config_.eval_time_interval)
  {
    SpeedPoint speed_point;
    heuristic_speed_data_.EvaluateByTime(time_stamp, &speed_point);
    double ref_s = speed_point.s + init_sl_point_.s;
    if (ref_s < start_s)
    {
      continue;
    }
    if (ref_s > end_s)
    {
      break;
    }

    const double s = ref_s - start_s; // s on spline curve
    const double l = curve.Evaluate(0, s);
    const double dl = curve.Evaluate(1, s);

    const SLPoint sl = util::PointFactory::ToSLPoint(ref_s, l);
    const Box2d ego_box = GetBoxFromSLPoint(sl, dl);
    // 当前时刻下，与所有动态dynamic_obstacle的cost
    for (const auto &obstacle_trajectory : dynamic_obstacle_boxes_)
    {
      obstacle_cost += GetCostBetweenObsBoxes(ego_box, obstacle_trajectory.at(index));
    }
  }
  static constexpr double kDynamicObsWeight = 1e-6;
  obstacle_cost.safety_cost *= (Config_.eval_time_interval * kDynamicObsWeight);
  return obstacle_cost;
}

bool TrajectoryCost::IsOffRoad(const double ref_s, const double l, const double dl, const bool is_change_lane_path)
{
  static constexpr double kIgnoreDistance = 5.0;
  if (ref_s - init_sl_point_.s < kIgnoreDistance)
  {
    return false;
  }
  Vec2d rear_center(0.0, l);
  Vec2d vec_to_center((Config_.front_edge_to_center - Config_.back_edge_to_center) / 2.0,
                      (Config_.left_edge_to_center - Config_.right_edge_to_center) / 2.0);
  Vec2d rear_center_to_center = vec_to_center.rotate(std::atan(dl));
  Vec2d center = rear_center + rear_center_to_center;
  Vec2d front_center = center + rear_center_to_center;

  const double buffer = 0.1; // in meters
  const double r_w = (Config_.left_edge_to_center + Config_.right_edge_to_center) / 2.0;
  const double r_l = Config_.back_edge_to_center;
  const double r = std::sqrt(r_w * r_w + r_l * r_l);

  // 原本是要从参考轨迹得到道路左右边界，道路宽度，这里我们假设道路都一样的宽度
  // double left_width = 0.0;
  // double right_width = 0.0;
  // reference_line_->GetLaneWidth(ref_s, &left_width, &right_width);
  double left_width = Config_.FLAGS_default_reference_line_width / 2.0;
  double right_width = Config_.FLAGS_default_reference_line_width / 2.0;

  double left_bound = std::max(init_sl_point_.l + r + buffer, left_width);
  double right_bound = std::min(init_sl_point_.l - r - buffer, -right_width);
  if (rear_center.y() + r + buffer / 2.0 > left_bound || rear_center.y() - r - buffer / 2.0 < right_bound)
  {
    return true;
  }
  if (front_center.y() + r + buffer / 2.0 > left_bound || front_center.y() - r - buffer / 2.0 < right_bound)
  {
    return true;
  }

  return false;
}

/*
结论1：静态障碍物开销其实思路和路径开销一样，在这条多项式曲线上采样，计算每个采样点和所有障碍物的标定框是否重叠，重叠cost就比较大。分情况：
    1：障碍物和无人车侧方向上的距离大于一个阈值(FLAGS_lateral_ignore_buffer=0.5,可调)，那么cost就是0，忽略障碍物
    2：如果障碍物在无人车后方，cost为0，可忽略。否则就计算cost
结论2：从两个方向计算的方法来看，我们可以看到Sigmoid(·)这个函数是单调递减的，在0.5(obstacle_collision_distance)时取0.5，越大取值越小。
所以Apollo鼓励无人车在侧方向和前方向上与障碍物保持一定距离，如0.5m以上。

*/
ComparableCost TrajectoryCost::GetCostFromObsSL(const double adc_s, const double adc_l,
                                                const SL_Boundary &obs_sl_boundary)
{
  ComparableCost obstacle_cost;
  if (obs_sl_boundary.start_l_ * obs_sl_boundary.end_l_ <= 0.0)
  {
    return obstacle_cost;
  }

  const double adc_front_s = adc_s + Config_.front_edge_to_center;
  const double adc_end_s = adc_s - Config_.back_edge_to_center;
  const double adc_left_l = adc_l + Config_.left_edge_to_center;
  const double adc_right_l = adc_l - Config_.right_edge_to_center;

  if (adc_left_l + Config_.FLAGS_lateral_ignore_buffer < obs_sl_boundary.start_l_ ||
      adc_right_l - Config_.FLAGS_lateral_ignore_buffer > obs_sl_boundary.end_l_)
  {
    return obstacle_cost;
  }

  bool no_overlap =
      ((adc_front_s < obs_sl_boundary.start_s_ || adc_end_s > obs_sl_boundary.end_s_) ||             // longitudinal
       (adc_left_l + 0.1 < obs_sl_boundary.start_l_ || adc_right_l - 0.1 > obs_sl_boundary.end_l_)); // lateral

  if (!no_overlap)
  {
    obstacle_cost.cost_items[ComparableCost::HAS_COLLISION] = true;
  }

  // if obstacle is behind ADC, ignore its cost contribution.
  if (adc_front_s > obs_sl_boundary.end_s_)
  {
    return obstacle_cost;
  }

  // 静态障碍物中心和无人车中心侧方向上的距离
  const double delta_l = std::fmax(adc_right_l - obs_sl_boundary.end_l_, obs_sl_boundary.start_l_ - adc_left_l);
  /*
  AWARN << "adc_s: " << adc_s << "; adc_left_l: " << adc_left_l
        << "; adc_right_l: " << adc_right_l << "; delta_l = " << delta_l;
  AWARN << obs_sl_boundary.ShortDebugString();
  */

  static constexpr double kSafeDistance = 0.6;
  if (delta_l < kSafeDistance)
  {
    //  侧方向距离造成的cost
    obstacle_cost.safety_cost += Config_.obstacle_collision_cost * common::math::Sigmoid(Config_.obstacle_collision_distance - delta_l);
  }

  return obstacle_cost;
}

// Simple version: calculate obstacle cost by distance
ComparableCost TrajectoryCost::GetCostBetweenObsBoxes(const Box2d &ego_box, const Box2d &obstacle_box) const
{
  ComparableCost obstacle_cost;

  const double distance = obstacle_box.DistanceTo(ego_box);
  if (distance > Config_.obstacle_ignore_distance)
  {
    return obstacle_cost;
  }
  // A. 计算碰撞cost
  obstacle_cost.safety_cost += Config_.obstacle_collision_cost * common::math::Sigmoid(Config_.obstacle_collision_distance - distance);
  obstacle_cost.safety_cost += 20.0 * common::math::Sigmoid(Config_.obstacle_risk_distance - distance);
  return obstacle_cost;
}

Box2d TrajectoryCost::GetBoxFromSLPoint(const SLPoint &sl, const double dl) const
{
  Vec2d xy_point;
  reference_line_->SLToXY(sl, &xy_point);

  ReferencePoint reference_point = reference_line_->GetReferencePoint(sl.s);

  const double one_minus_kappa_r_d = 1 - reference_point.kappa() * sl.l;
  const double delta_theta = std::atan2(dl, one_minus_kappa_r_d);
  const double theta = common::math::NormalizeAngle(delta_theta + reference_point.heading());
  return Box2d(xy_point, theta, vehicle_config_.vehicle_length, vehicle_config_.vehicle_width);
}