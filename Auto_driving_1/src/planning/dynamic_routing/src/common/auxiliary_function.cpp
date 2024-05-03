#include "auxiliary_function.h"


//返回：最近的点
std::pair<double, double>
Auxiliary::Find_nearest_piont(std::pair<double, double> ObastaclePoint,
                              std::pair<std::vector<double>, std::vector<double>> reference_path_, int& index_ob)
{
  std::pair<double, double> nearest;
  int index = 0;
  double min_val = 1000;
  double distance;
  for (size_t i = 0; i < reference_path_.first.size(); i++)
  {
    distance = sqrt(pow(ObastaclePoint.first - reference_path_.first[i], 2) +
                    pow(ObastaclePoint.second - reference_path_.second[i], 2));
    if (distance < min_val)
    {
      min_val = distance;  //更新
      index = i;
    }
  }
  index_ob = index;
  nearest.first = reference_path_.first[index];
  nearest.second = reference_path_.second[index];
  return nearest;
}

//根据x,y求s
double Auxiliary::find_s(CubicSpline2D* csp, double x, double y, double s0, std::vector<double> s)
{
  double s_closest = s0;
  double closest = INFINITY;
  double si = s.front();
  do
  {
    double px = csp->calc_x(si);
    double py = csp->calc_y(si);
    double dist = sqrt(pow((x - px), 2) + pow((y - py), 2));
    if (dist < closest)
    {
      closest = dist;
      s_closest = si;
    }
    si += 0.1;
  } while (si < s.back());
  return s_closest;
}

//传入：障碍物中心点ObastaclePoint， 最近的参考点rxy
//输出：横向偏移
double Auxiliary::cartesian_to_frenet(std::pair<double, double> ObastaclePoint, std::pair<double, double> rxy,
                                      double rtheta)
{
  const double dx = ObastaclePoint.first - rxy.first;
  const double dy = ObastaclePoint.second - rxy.second;

  const double cos_theta_r = std::cos(rtheta);
  const double sin_theta_r = std::sin(rtheta);

  const double cross_rd_nd = cos_theta_r * dy - sin_theta_r * dx;

  double ptr_d = std::copysign(std::sqrt(dx * dx + dy * dy), cross_rd_nd);

  return ptr_d;
}