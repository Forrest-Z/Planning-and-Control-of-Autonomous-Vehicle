#include "Obstacle.h"
#include "path_struct.h"
#include <string>

// 求障碍物顶点
PPoint ob_left_front;
PPoint ob_left_buttom;
PPoint ob_right_front;
PPoint ob_right_buttom;

GetObstacle::GetObstacle()
{
  ros::NodeHandle nh_;
  AllObstacle.clear();

  // 订阅感知的障碍物
  obstacle = nh_.subscribe<object_msgs::DynamicObjectArray>("/xsj/obstacle/obstacles", 10, &GetObstacle::setObstacles, this);
}

// 回调，获得障碍物信息
/*
说明：这里的障碍物信息可能根你们感知模块传过来的不一样
*/
void GetObstacle::setObstacles(const object_msgs::DynamicObjectArray::ConstPtr &msgs)
{
  /*---------------------------------------获取障碍物信息----------------------------------------*/
  if (msgs->objects.size() > 0) // 如果感知有识别到障碍物
  {
    AllObstacle.clear(); // 每次接受障碍物的时候，清空，更新
    // 存到AllObstacle
    for (size_t i = 0; i < msgs->objects.size(); i++)
    {
      Obstacle obs;
      /*-------------------------------------每种形状都有的基本信息----------------------------------------*/
      // 中心点
      obs.centerpoint.position.x = msgs->objects[i].state.pose_covariance.pose.position.x;
      obs.centerpoint.position.y = msgs->objects[i].state.pose_covariance.pose.position.y;
      obs.centerpoint.position.z = 0; // 压缩二维
      // std::cout << "position:" << obs.centerpoint.position.x << "," << obs.centerpoint.position.y<< "\n";
      obs.obstacle_id = msgs->objects[i].id;              // id
      obs.obstacle_type = msgs->objects[i].semantic.type; // 类型
      obs.obstacle_shape = msgs->objects[i].shape.type;   // 形状
      // 朝向(要根感知确认一下)
      obs.obstacle_threa = tf::getYaw(msgs->objects[i].state.pose_covariance.pose.orientation);
      // 时间戳
      obs.timestamp_ = msgs->header.stamp;
      // 线速度
      obs.obstacle_velocity = msgs->objects[i].state.twist_covariance.twist.linear.x;
      // 加速度
      obs.obstacle_acc = 0;
      // 动态障碍物预测轨迹
      if (obs.obstacle_velocity > 0.2) // 动态障碍物
      {
        oba.Generater_Trajectory(obs.trajectory_prediction, obs.centerpoint,
                                 Config_.dynamic_obs_predict_time, obs.obstacle_threa, obs.obstacle_velocity, obs.obstacle_acc);
      }
      /*--------------------------------------不同形状有差别的信息-----------------------------------------*/

      if (msgs->objects[i].shape.type == 0) // 方形
      {
        obs.pinnacle.poses.clear(); // 顶点暂时为空
        obs.obstacle_radius =
            sqrt(pow(msgs->objects[i].shape.dimensions.x, 2) + pow(msgs->objects[i].shape.dimensions.y, 2)) / 2;
        // 长和宽
        obs.obstacle_length = msgs->objects[i].shape.dimensions.y;
        obs.obstacle_width = msgs->objects[i].shape.dimensions.x;
        obs.obstacle_height = msgs->objects[i].shape.dimensions.z;
      }
      else if (msgs->objects[i].shape.type == 1) // 圆柱体
      {
        obs.obstacle_radius = msgs->objects[i].shape.dimensions.x; // 半径
        // 顶点
        obs.pinnacle.poses.clear();
        PPoint ob_center(obs.centerpoint.position.x, obs.centerpoint.position.y);
        oba.CalculateCarBoundaryPoint(1, 1, ob_center, obs.obstacle_threa, ob_left_front, ob_left_buttom,
                                      ob_right_buttom, ob_right_front);
        // oba.visualization_points(ob_left_front, ob_left_buttom, ob_right_buttom, ob_right_front);//显示障碍物顶点

        std::vector<PPoint> ob_vector;
        ob_vector.emplace_back(ob_right_front);  // 右下角
        ob_vector.emplace_back(ob_left_buttom);  // 右上角
        ob_vector.emplace_back(ob_left_front);   // 左上角
        ob_vector.emplace_back(ob_right_buttom); // 左下角
        for (size_t j = 0; j < ob_vector.size(); j++)
        {
          geometry_msgs::Pose sds;
          sds.position.x = ob_vector[j].x;
          sds.position.y = ob_vector[j].y;
          sds.position.z = 0; // 压缩二维
          obs.pinnacle.poses.push_back(sds);
          obs.polygon_points.push_back(Vec2d(ob_vector[j].x, ob_vector[j].y));
        }
        // 长和宽（假设）
        obs.obstacle_length = msgs->objects[i].shape.dimensions.x;
        obs.obstacle_width = msgs->objects[i].shape.dimensions.y;
        obs.obstacle_height = msgs->objects[i].shape.dimensions.z;
      }
      else if (msgs->objects[i].shape.type == 2) // 多边形:未知
      {
        // 顶点
        obs.pinnacle.poses.clear();
        for (size_t j = 0; j < msgs->objects[i].shape.footprint.points.size(); j++)
        {
          geometry_msgs::Pose sds;
          sds.position.x = msgs->objects[i].shape.footprint.points[j].x;
          sds.position.y = msgs->objects[i].shape.footprint.points[j].y;
          sds.position.z = 0; // 压缩二维
          obs.pinnacle.poses.push_back(sds);
          obs.polygon_points.push_back(Vec2d(msgs->objects[i].shape.footprint.points[j].x, msgs->objects[i].shape.footprint.points[j].y));
        }
        // 半径为0
        obs.obstacle_radius = 0;
        // 长和宽（假设）
        obs.obstacle_length = msgs->objects[i].shape.dimensions.x;
        obs.obstacle_width = msgs->objects[i].shape.dimensions.y;
        obs.obstacle_height = msgs->objects[i].shape.dimensions.z;
      }
      AllObstacle.push_back(obs);
    }
  }
}

void Obstacle::SetSLBoundary(SL_Boundary &sl_boundary) // Lattice专用
{
  sl_boundary_ = std::move(sl_boundary);
}
void Obstacle::SetSTBoundary(ST_Boundary &st_boundary) // Lattice专用
{
  path_st_boundary_ = std::move(st_boundary);
}
const SL_Boundary &Obstacle::PerceptionSLBoundary() const
{
  return sl_boundary_;
}
const ST_Boundary &Obstacle::path_st_boundary() const
{
  return path_st_boundary_;
}

bool Obstacle::IsStatic() const
{
  return obstacle_velocity < 0.2;
}
bool Obstacle::IsVirtual() const
{
  return false; // 假设都不是虚拟的
}

bool Obstacle::HasTrajectory() const
{
  return !(trajectory_prediction.trajectory_point_size() == 0); // 没有预测轨迹就是静态障碍物
}

const ObjectDecisionType &Obstacle::LongitudinalDecision() const
{
  return longitudinal_decision_;
}

void Obstacle::SetLongitudinalDecision(ObjectNudge::Type type)
{
  longitudinal_decision_.nudge_.type = type;
}

void Obstacle::SetLateralDecision()
{
  if (sl_boundary_.start_l_ > 0) // 障碍物在左边，EM专用
  {
    lateral_decision_.nudge_.type = ObjectNudge::Type::RIGHT_NUDGE;
  }
  if (sl_boundary_.end_l_ < 0) // 障碍物在右边，EM专用
  {
    lateral_decision_.nudge_.type = ObjectNudge::Type::LEFT_NUDGE;
  }
  if (sl_boundary_.end_l_ > -Config_.FLAGS_numerical_epsilon && sl_boundary_.start_l_ < Config_.FLAGS_numerical_epsilon)
  {
    lateral_decision_.nudge_.type = ObjectNudge::Type::NO_NUDGE;
  }
}

const ObjectDecisionType &Obstacle::LateralDecision() const
{
  return lateral_decision_;
}
bool Obstacle::HasLateralDecision() const
{
  return lateral_decision_.object_tag_case() != ObjectDecisionType::Type::OBJECT_TAG_NOT_SET;
}

bool Obstacle::HasLongitudinalDecision() const
{
  return longitudinal_decision_.object_tag_case() != ObjectDecisionType::Type::OBJECT_TAG_NOT_SET;
}

Box2d Obstacle::PerceptionBoundingBox() const
{
  // 必须在订阅之后调用
  return Box2d({centerpoint.position.x, centerpoint.position.y}, obstacle_threa, obstacle_length, obstacle_width);
}

Box2d Obstacle::GetBoundingBox(const TrajectoryPoint &point) const
{
  return Box2d({point.x, point.y}, point.theta, obstacle_length,
               obstacle_width);
}

// 获得障碍物在当前时刻的TrajectoryPoint,为了求GetBoundingBox
TrajectoryPoint Obstacle::GetPointAtTime(const double relative_time) const
{
  const auto &points = trajectory_prediction.trajectory_point();
  // std::cout<<"points.size():"<<points.size()<<"\n";
  if (points.size() < 2) // 认为是静态障碍物
  {
    TrajectoryPoint point;
    point.set_x(centerpoint.position.x);
    point.set_y(centerpoint.position.y);
    point.set_z(0);
    point.set_theta(obstacle_threa);
    point.set_s(0.0);
    point.set_kappa(0.0);
    point.set_dkappa(0.0);
    point.set_v(0.0);
    point.set_a(0.0);
    point.set_relative_time(0.0);
    return point;
  }
  else // 认为是一个运动的障碍物
  {
    for (size_t i = 0; i < points.size(); i++)
    {
      if (points[i].has_path_point() == false)
      {
        std::cout << "has_path_point_:" << i << ","
                  << "x:" << points[i].x << ","
                  << "y:" << points[i].y << "\n";
      }
    }
    if (relative_time >= points.back().relative_time)
    {
      return points.back();
    }

    auto comp = [](const TrajectoryPoint p, const double time) {
      return p.relative_time < time;
    };

    auto it_lower = std::lower_bound(points.begin(), points.end(), relative_time, comp);
    if (it_lower == points.begin())
    {
      return points.front();
    }
    else if (it_lower == points.end())
    {
      return points.back();
    }
    return common::math::InterpolateUsingLinearApproximation(*(it_lower - 1), *it_lower, relative_time);
  }
}

const common::math::Polygon2d &Obstacle::PerceptionPolygon() const
{
  return perception_polygon_;
}