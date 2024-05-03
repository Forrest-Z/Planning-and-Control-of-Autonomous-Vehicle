#pragma once
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <string>
#include <vector>
#include <fstream>
#include "reference_line.h"
#include "CubicSpline2D.h"
#include "path_matcher.h"
#include "path_time_graph.h"

class MultilanePlanner
{
public:
    MultilanePlanner() = default;
    ~MultilanePlanner() = default;
    void visual_reference_lines(std::string reference, nav_msgs::Path &reference_lines);
    void generate_reference_lines(std::string reference,
                                  std::vector<double> &accumulated_s,
                                  std::vector<ReferencePoint> &reference_points);
    std::pair<double, double> get_EgoLonForReference_other(const InitialConditions &planning_init_point,
                                                           const std::vector<ReferencePoint> &discretized_ref_points);
    std::pair<double, double> get_Stoppoint(const geometry_msgs::Pose &stop_point,
                                            const std::vector<ReferencePoint> &discretized_ref_points);

    bool CanChangeLaneToReference_other(std::shared_ptr<PathTimeGraph> &ptr_path_time_graph, double ego_s_other);
};
