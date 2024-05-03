/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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
#include "path_data.h"
#include "point_factory.h"

PathData::PathData(const ReferenceLine &reference_line) : reference_line_(reference_line)
{
  // std::cout << "referenceLine:" << reference_line_.test(1) << "\n";
}

bool PathData::SetDiscretizedPath(DiscretizedPath path)
{
  discretized_path_ = std::move(path);
  if (!XYToSL(discretized_path_, &frenet_path_))
  {
    // std::cout << "Fail to transfer discretized path to frenet path.";
    return false;
  }
  // DCHECK_EQ(discretized_path_.size(), frenet_path_.size());
  path_data_history_.push_back(std::make_pair(discretized_path_, frenet_path_));
  return true;
}

bool PathData::SetFrenetPath(FrenetFramePath frenet_path)
{
  frenet_path_ = std::move(frenet_path);

  // std::cout << "frenet_path_:" << frenet_path_.Size() << "\n";

  if (!SLToXY(frenet_path_, &discretized_path_))
  {
    // std::cout << "Fail to transfer frenet path to discretized path.";
    return false;
  }
  // DCHECK_EQ(discretized_path_.size(), frenet_path_.size());
  path_data_history_.push_back(std::make_pair(discretized_path_, frenet_path_));
  return true;
}

bool PathData::SetPathPointDecisionGuide(
    std::vector<std::tuple<double, PathPointType, double>> path_point_decision_guide)
{
  if (frenet_path_.empty() || discretized_path_.empty())
  {
    // std::cout << "Should NOT set path_point_decision_guide when frenet_path or "
    //              "world frame trajectory is empty. ";
    return false;
  }
  path_point_decision_guide_ = std::move(path_point_decision_guide);
  return true;
}

const DiscretizedPath &PathData::discretized_path() const
{
  return discretized_path_;
}

const FrenetFramePath &PathData::frenet_frame_path() const
{
  return frenet_path_;
}

std::list<std::pair<DiscretizedPath, FrenetFramePath>> &PathData::path_data_history()
{
  return path_data_history_;
}

const std::vector<std::tuple<double, PathData::PathPointType, double>> &PathData::path_point_decision_guide() const
{
  return path_point_decision_guide_;
}

bool PathData::Empty() const
{
  return discretized_path_.empty() && frenet_path_.empty();
}

int PathData::Size() const
{
  return discretized_path_.Size();
}
double PathData::Length() const
{
  return frenet_path_.back().s;
}

// void PathData::SetReferenceLine(const ReferenceLine* reference_line)
// {
//   reference_line_ = reference_line;
// }

PathPoint PathData::GetPathPointWithPathS(const double s) const
{
  return discretized_path_.Evaluate(s);
}

FrenetFramePoint PathData::GetFrenetFramePointWithPathS(const double s) const
{
  return frenet_path_.EvaluateByS(s);
}

bool PathData::GetPathPointWithRefS(const double ref_s, PathPoint *const path_point) const
{
  // ACHECK(reference_line_);
  // DCHECK_EQ(discretized_path_.size(), frenet_path_.size());
  if (ref_s < 0)
  {
    std::cout << "ref_s[" << ref_s << "] should be > 0";
    return false;
  }
  if (ref_s > frenet_path_.back().s)
  {
    // std::cout << "ref_s is larger than the length of frenet_path_ length [" << frenet_path_.back().s << "].";
    return false;
  }

  uint32_t index = 0;
  const double kDistanceEpsilon = 1e-3;
  for (uint32_t i = 0; i + 1 < frenet_path_.size(); ++i)
  {
    if (fabs(ref_s - frenet_path_.at(i).s) < kDistanceEpsilon)
    {
      // path_point->CopyFrom(discretized_path_.at(i));
      return true;
    }
    if (frenet_path_.at(i).s < ref_s && ref_s <= frenet_path_.at(i + 1).s)
    {
      index = i;
      break;
    }
  }
  double r = (ref_s - frenet_path_.at(index).s) / (frenet_path_.at(index + 1).s - frenet_path_.at(index).s);

  const double discretized_path_s =
      discretized_path_.at(index).s + r * (discretized_path_.at(index + 1).s - discretized_path_.at(index).s);
  // path_point->CopyFrom(discretized_path_.Evaluate(discretized_path_s));

  return true;
}

void PathData::Clear()
{
  discretized_path_.clear();
  frenet_path_.clear();
  path_point_decision_guide_.clear();
}

bool PathData::SLToXY(const FrenetFramePath &frenet_path, DiscretizedPath *const discretized_path)
{
  std::vector<PathPoint> path_points;
  for (const FrenetFramePoint &frenet_point : frenet_path)
  {
    // std::cout << "(" << frenet_point.s << "," << frenet_point.d << ")"
    //           << "\n";

    const SLPoint sl_point = util::PointFactory::ToSLPoint(frenet_point.s, frenet_point.d);
    Vec2d cartesian_point;

    if (!reference_line_.SLToXY(sl_point, &cartesian_point))
    {
      // std::cout << "Fail to convert sl point to xy point";
      return false;
    }

    const ReferencePoint ref_point = reference_line_.GetReferencePoint(frenet_point.s);
    const double theta = CartesianFrenetConverter::CalculateTheta(ref_point.heading(), ref_point.kappa(),
                                                                  frenet_point.d, frenet_point.d_d);
    // std::cout << "frenet_point: " << frenet_point.ShortDebugString();
    const double kappa = CartesianFrenetConverter::CalculateKappa(ref_point.kappa(), ref_point.dkappa(), frenet_point.d,
                                                                  frenet_point.d_d, frenet_point.d_dd);

    // std::cout << "(" << cartesian_point.x() << "," << cartesian_point.y() << ")"
    //           << "\n";
    double s = 0.0;
    double dkappa = 0.0;
    if (!path_points.empty())
    {
      Vec2d last = util::PointFactory::ToVec2d(path_points.back());
      const double distance = (last - cartesian_point).Length();
      s = path_points.back().s + distance;
      dkappa = (kappa - path_points.back().kappa) / distance;
    }
    path_points.push_back(
        util::PointFactory::ToPathPoint(cartesian_point.x(), cartesian_point.y(), 0.0, s, theta, kappa, dkappa));
  }
  // std::cout << "----------------------------------"
  //           << "\n";
  *discretized_path = DiscretizedPath(std::move(path_points));

  return true;
}

bool PathData::XYToSL(const DiscretizedPath &discretized_path, FrenetFramePath *const frenet_path)
{
  std::vector<FrenetFramePoint> frenet_frame_points;
  const double max_len = reference_line_.Length();
  for (const auto &path_point : discretized_path)
  {
    FrenetFramePoint frenet_point = reference_line_.GetFrenetPoint(path_point);
    if (!frenet_point.has_s())
    {
      SLPoint sl_point;
      if (!reference_line_.XYToSL(path_point, &sl_point))
      {
        std::cout << "Fail to transfer cartesian point to frenet point.";
        return false;
      }
      FrenetFramePoint frenet_point;
      // NOTICE: does not set dl and ddl here. Add if needed.
      frenet_point.set_s(std::max(0.0, std::min(sl_point.s, max_len)));
      frenet_point.set_l(sl_point.l);
      frenet_frame_points.push_back(std::move(frenet_point));
      continue;
    }
    frenet_point.set_s(std::max(0.0, std::min(frenet_point.s, max_len)));
    frenet_frame_points.push_back(std::move(frenet_point));
  }
  *frenet_path = FrenetFramePath(std::move(frenet_frame_points));
  return true;
}

void PathData::set_path_label(const std::string &label)
{
  path_label_ = label;
}

const std::string &PathData::path_label() const
{
  return path_label_;
}

const std::vector<PathPoint> &PathData::path_reference() const
{
  return path_reference_;
}

void PathData::set_path_reference(const std::vector<PathPoint> &path_reference)
{
  path_reference_ = std::move(path_reference);
}