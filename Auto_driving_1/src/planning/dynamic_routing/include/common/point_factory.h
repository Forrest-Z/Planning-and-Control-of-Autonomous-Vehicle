#pragma once
#include "path_points.h"

namespace util
{
  class PointFactory
  {
  public:
    template <typename XY>
    static inline Vec2d ToVec2d(const XY &xy)
    {
      return Vec2d(xy.x, xy.y);
    }

    static inline SLPoint ToSLPoint(const double s, const double l)
    {
      SLPoint sl;
      sl.set_s(s);
      sl.set_l(l);
      return sl;
    }

    static inline Vec2d ToPointENU(const double x, const double y)
    {
      return Vec2d(x, y);
    }

    static inline SpeedPoint ToSpeedPoint(const double s, const double t, const double v = 0, const double a = 0,
                                          const double da = 0)
    {
      SpeedPoint speed_point;
      speed_point.set_s(s);
      speed_point.set_t(t);
      speed_point.set_v(v);
      speed_point.set_a(a);
      speed_point.set_da(da);
      return speed_point;
    }

    static inline PathPoint ToPathPoint(const double x, const double y, const double z, const double s,
                                        const double theta, const double kappa, const double dkappa)
    {
      PathPoint path_point;
      path_point.set_x(x);
      path_point.set_y(y);
      path_point.set_z(z);
      path_point.set_s(s);
      path_point.set_theta(theta);
      path_point.set_kappa(kappa);
      path_point.set_dkappa(dkappa);
      return path_point;
    }

    static inline PathPoint MakePathPoint(const double x, const double y, const double z,
                                          const double theta, const double kappa,
                                          const double dkappa)
    {
      PathPoint path_point;
      path_point.set_x(x);
      path_point.set_y(y);
      path_point.set_z(z);
      path_point.set_theta(theta);
      path_point.set_kappa(kappa);
      path_point.set_dkappa(dkappa);
      return path_point;
    }
  };

} // namespace util