#include "dp_poly_path_optimizer.h"
/*-------------------------------------------------DP规划入口---------------------------------------------------*/
DpPolyPathOptimizer::DpPolyPathOptimizer(const std::vector<const Obstacle *> &obstacles,
                                         const nav_msgs::Odometry &vehicle_pos)
    : obstacles_(obstacles), vehicle_pos_(vehicle_pos) //传入障碍物和车的里程计
{
}
DpPolyPathOptimizer::DpPolyPathOptimizer()
{
}
DpPolyPathOptimizer::~DpPolyPathOptimizer()
{
}

/*
DP-Path的主要思路是以自车当前位置为起点，沿着车道横纵向（横向为L或d，纵向为s，这里为了字母更清晰，横向统一以d表示）采样一些点，
横向采样的一组点叫做level，点封装成node后，分别计算不同level间的node的cost，就构成了graph。利用DP更新node的最小cost，便找
到了代价最小的一条路径。听起来很像传统的图搜索方法找最短路径，区别在于cost包含了平滑、无碰的指标。
*/

//处理结果：最短路径就是依据若干level之间分段5次多项式的采样点，保存在path_data.frenet_path_（SL系）和path_data.discretized_path_（XY系）中
bool DpPolyPathOptimizer::Process(const SpeedData &speed_data, const ReferenceLine &reference_line,
                                  const TrajectoryPoint &init_point, PathData *const path_data)
{
  DpRoadGraph dp_road_graph(speed_data, reference_line, vehicle_pos_);
  dp_road_graph.SetWaypointSampler(new WaypointSampler());

  // FindPathTunnel()主要分为3部分：先设置相关前提条件，然后查找代价最小路径，
  //最后对每段代价最小路径采样以构造FrenetFramePath类的实例，并存入path_data中。
  if (!dp_road_graph.FindPathTunnel(init_point, obstacles_, path_data))
  {
    ROS_INFO("Failed to find tunnel in road graph");
    return false;
  }

  return true;
}