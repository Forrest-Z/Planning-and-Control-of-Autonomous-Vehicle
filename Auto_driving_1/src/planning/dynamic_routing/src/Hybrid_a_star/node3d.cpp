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
 **/

#include "node3d.h"

// A*中的node单纯的指一个节点或一个状态，一般是其（x，y）坐标。这里的Node3d不同，
//除了包含一个点坐标（x_，y_，phi_）之外，还包含了一串这样的坐标集合。
Node3d::Node3d(double x, double y, double phi)
{
    x_ = x;
    y_ = y;
    phi_ = phi;
}

Node3d::Node3d(double x, double y, double phi,
               const std::vector<double> &XYbounds)
{
    // CHECK_EQ(XYbounds.size(), 4) << "XYbounds size is not 4, but" << XYbounds.size();

    x_ = x;
    y_ = y;
    phi_ = phi;

    x_grid_ = static_cast<int>(
        (x_ - XYbounds[0]) / Config_.xy_grid_resolution);
    y_grid_ = static_cast<int>(
        (y_ - XYbounds[2]) / Config_.xy_grid_resolution);
    phi_grid_ = static_cast<int>(
        (phi_ - (-M_PI)) / Config_.phi_grid_resolution);

    traversed_x_.push_back(x);
    traversed_y_.push_back(y);
    traversed_phi_.push_back(phi);

    index_ = ComputeStringIndex(x_grid_, y_grid_, phi_grid_);
}

/*
从下面的构造函数可以看出，（x_，y_，phi_）点就是这一串点中的最后一个。其实，这些点都是在1个grid内的。
即，1个grid包含1个Node3d，1个Node3d包含了以（x_，y_，phi_）为终点、同在1个grid内的、一串路径点集的信息。
*/
Node3d::Node3d(const std::vector<double> &traversed_x,
               const std::vector<double> &traversed_y,
               const std::vector<double> &traversed_phi,
               const std::vector<double> &XYbounds)
{
    // CHECK_EQ(XYbounds.size(), 4) << "XYbounds size is not 4, but" << XYbounds.size();
    // CHECK_EQ(traversed_x.size(), traversed_y.size());
    // CHECK_EQ(traversed_x.size(), traversed_phi.size());

    x_ = traversed_x.back();
    y_ = traversed_y.back();
    phi_ = traversed_phi.back();

    // XYbounds in xmin, xmax, ymin, ymax
    x_grid_ = static_cast<int>(
        (x_ - XYbounds[0]) / Config_.xy_grid_resolution);
    y_grid_ = static_cast<int>(
        (y_ - XYbounds[2]) / Config_.xy_grid_resolution);
    phi_grid_ = static_cast<int>(
        (phi_ - (-M_PI)) / Config_.phi_grid_resolution);

    traversed_x_ = traversed_x;
    traversed_y_ = traversed_y;
    traversed_phi_ = traversed_phi;

    index_ = ComputeStringIndex(x_grid_, y_grid_, phi_grid_);
    step_size_ = traversed_x.size();
}

Box2d Node3d::GetBoundingBox(const double x, const double y, const double phi)
{
    double ego_length = Config_.FLAGS_vehicle_length;
    double ego_width = Config_.FLAGS_vehicle_width;
    double shift_distance =
        ego_length / 2.0 - Config_.back_edge_to_center;
    Box2d ego_box(
        {x + shift_distance * std::cos(phi), y + shift_distance * std::sin(phi)},
        phi, ego_length, ego_width);
    return ego_box;
}

bool Node3d::operator==(const Node3d &right) const
{
    return right.GetIndex() == index_;
}

std::string Node3d::ComputeStringIndex(int x_grid, int y_grid, int phi_grid)
{
    std::string res = "";
    std::string res1 = std::to_string(x_grid);
    std::string res2 = std::to_string(y_grid);
    std::string res3 = std::to_string(phi_grid);

    res = res1 + "_" + res2 + "_" + res3;

    return res;
}
