#pragma once
#include <string>
struct Point3D_s
{
  double x;
  double y;
  double z;
};

struct State_s
{
  double x;
  double y;
  double z;
  double pitch;
  double roll;
  double yaw;
};
struct Point4d_s
{
  double x;
  double y;
  double z;
  double o;
};
struct Point2d_s
{
  double x;
  double y;
};
//参考系的定义
const std::string Frame_id = "map";
