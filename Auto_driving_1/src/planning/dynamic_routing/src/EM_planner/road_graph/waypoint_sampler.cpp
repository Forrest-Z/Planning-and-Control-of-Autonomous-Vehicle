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

#include "waypoint_sampler.h"
#include "point_factory.h"
#include "math_utils.h"
void WaypointSampler::Init(const VehicleConfig &vehicle_config, const ReferenceLine *reference_line,
                           const SLPoint &init_sl_point, const FrenetFramePoint &init_frenet_frame_point)
{
  reference_line_ = reference_line;
  vehicle_config_ = vehicle_config;
  init_sl_point_ = init_sl_point;
  init_frenet_frame_point_ = init_frenet_frame_point;
}

//在s-l图上采样，s看作横坐标，l看作总坐标，（0，0）是自主车的起始点
bool WaypointSampler::SamplePathWaypoints(const TrajectoryPoint &init_point,
                                          std::vector<std::vector<SLPoint>> *const points)
{
  points->clear();
  points->insert(points->begin(), std::vector<SLPoint>{init_sl_point_}); //先把起点存起来

  // std::cout << " s:" << init_sl_point_.s << ","
  //           << " l:" << init_sl_point_.l << "\n";

  // std::cout << " reference_line_->Length():" << reference_line_->Length() << "\n";

  //纵向s: 以8秒的窗口来撒点，当init_point_v * 8小于40米时，以40米的范围撒点
  const double kMinSampleDistance = Config_.PathLength; // 前探采样距离,原本是40，但是我改成20，不想太长，注释里的40都是指这个
  // init_point.DiscretizedPath_.point.front().v:轨迹第一个点的速度
  // 确定要规划的纵向距离，8秒*init_v或40米
  const double total_length =
      std::fmin(init_sl_point_.s + std::fmax(init_point.v * 8.0, kMinSampleDistance), reference_line_->Length());

  // std::cout << "total_length:" << total_length << "\n";

  //横向l: 采样个数和范围选择
  //自主车的半宽度
  // std::cout << " init_point.v:" << init_point.v << "\n";
  // std::cout << "vehicle_width:" << vehicle_config_.vehicle_width << "\n";
  const double half_adc_width = vehicle_config_.vehicle_width / 2.0;
  //每个level的撒点数，即每一纵列撒点数，这里是选7个， navigation_mode是高速的
  const double num_sample_per_level =
      Config_.FLAGS_use_navigation_mode ? Config_.navigator_sample_num_each_level : Config_.sample_points_num_each_level;

  constexpr double kSamplePointLookForwardTime = 4.0;
  //每层level之间的距离，即纵向采样间隔，与速度有关
  const double level_distance =
      common::math::Clamp(init_point.v * kSamplePointLookForwardTime, Config_.step_length_min, Config_.step_length_max);

  // std::cout << "level_distance:" << level_distance << "\n";

  // 初始s
  double accumulated_s = init_sl_point_.s; //先等于起点
  double prev_s = accumulated_s;

  static constexpr size_t kNumLevel = 3; //固定level的层数，为3层
  for (size_t i = 0; i < kNumLevel && accumulated_s < total_length; ++i)
  {
    accumulated_s += level_distance; //
    // s增加时的越界处理
    if (accumulated_s + level_distance / 2.0 > total_length)
    {
      accumulated_s = total_length;
    }
    const double s = std::fmin(accumulated_s, total_length);
    // 最小采样间隔
    static constexpr double kMinAllowedSampleStep = 1.0;
    if (std::fabs(s - prev_s) < kMinAllowedSampleStep)
    {
      continue; //采样间隔过小则不进行处理
    }
    prev_s = s;

    // 计算纵向每个位置，有效的左右边界，kBoundaryBuff是无人车与车道线边界线的间距。需要保持无人车在车道内行驶。
    // 原本是要从参考轨迹得到道路左右边界，道路宽度，这里我们假设道路都一样的宽度
    double left_width = Config_.FLAGS_default_reference_line_width / 2.0;
    double right_width = Config_.FLAGS_default_reference_line_width / 2.0;

    //撒点时，横向l 的左右有效范围是减去车宽、膨胀区域（这里是0.2m）后的宽度
    constexpr double kBoundaryBuff = 0.20; // 膨胀区域
    // 计算实际的可撒点的左右边界，  减去车宽和膨胀区域
    const double eff_right_width = right_width - half_adc_width - kBoundaryBuff;
    const double eff_left_width = left_width - half_adc_width - kBoundaryBuff;

    double sample_right_boundary = -eff_right_width;
    double sample_left_boundary = eff_left_width;

    // ---------------------------------车道变换场景中L的启发式变换，暂不考虑------------------------------------//
    const double delta_dl = 1.2 / 20.0;
    // 换道场景的L变化量，（初始斜率+预设变化率delta_dl）* level_distance
    const double kChangeLaneDeltaL =
        common::math::Clamp(level_distance * (std::fabs(init_frenet_frame_point_.d) + delta_dl), 1.2, 3.5);

    double kDefaultUnitL = kChangeLaneDeltaL / (num_sample_per_level - 1);

    // 如果当前参考线是变道，且变道不安全(无人车前后一定距离内有障碍物)
    if (Config_.IsChangeLanePath && Config_.IsClearToChangeLane) //换道时的特殊处理
    {
      kDefaultUnitL = 1.0;
    }
    const double sample_l_range = kDefaultUnitL * (num_sample_per_level - 1); // 换道的l的采样范围，求出来是1.2

    // std::cout << "sample_l_range:" << sample_l_range << "\n";

    //速度 1miles = 1.6km, 20*1.6/3.6 = 8.9m/s
    constexpr double kLargeDeviationL = 1.75;
    constexpr double kTwentyMilesPerHour = 8.94; // 8.94
    //换道调用，初始的l大于kLargeDeviationL,说明车很靠边
    if (Config_.IsChangeLanePath || std::fabs(init_sl_point_.l) > kLargeDeviationL)
    {
      //如果自主车的起始速度大于kTwentyMilesPerHour
      if (vehicle_config_.vehicle_velocity > kTwentyMilesPerHour)
      {
        sample_right_boundary = std::fmin(-eff_right_width, init_sl_point_.l);
        sample_left_boundary = std::fmax(eff_left_width, init_sl_point_.l);

        if (init_sl_point_.l > eff_left_width)
        {
          sample_right_boundary = std::fmax(sample_right_boundary, init_sl_point_.l - sample_l_range);
        }
        if (init_sl_point_.l < eff_right_width)
        {
          sample_left_boundary = std::fmin(sample_left_boundary, init_sl_point_.l + sample_l_range);
        }
      }
    }

    // ------------------------------------------开始采样-----------------------------------------------//
    //要来保存每一个纵向level的横向l的采样啦
    std::vector<double> sample_l;
    // 换道情况下
    if (Config_.IsChangeLanePath && Config_.IsClearToChangeLane) // IsChangeLanePathAndNotSafe先设置false,因为我们还没变道
    {
      sample_l.push_back(0); //两条参考线才考虑OffsetToOtherReferenceLine，否则为0，我们这里先设置为0
    }
    else // 不是换道情况下，目前是这个
    {
      //从左右边界进行均匀切分，返回sample_l
      common::math::uniform_slice(sample_right_boundary, sample_left_boundary,
                                  static_cast<uint32_t>(num_sample_per_level - 1), &sample_l);
    }
    //一次把每列的点存入level_points
    std::vector<SLPoint> level_points;
    for (size_t j = 0; j < sample_l.size(); ++j)
    {
      SLPoint sl = util::PointFactory::ToSLPoint(s, sample_l[j]);
      level_points.push_back(std::move(sl)); //
    }
    if (!level_points.empty())
    {
      points->emplace_back(level_points);
    }
  }
  return true;
}
