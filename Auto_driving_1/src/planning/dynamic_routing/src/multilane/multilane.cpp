#include "multilane.h"

void MultilanePlanner::visual_reference_lines(std::string reference, nav_msgs::Path &reference_lines)
{
    std::string reference_lines_path = "";
    ros::param::get("referenceline_path", reference_lines_path);

    reference_lines.poses.clear();
    reference_lines.header.frame_id = Frame_id;
    reference_lines.header.stamp = ros::Time::now();

    std::string reference_lines_test_path = reference_lines_path + "/" + reference;

    std::ifstream infile;
    // 打开文件
    infile.open(reference_lines_test_path + "/x");
    // 读取数据
    double d;
    std::vector<double> sample_x;
    while (infile >> d)
        sample_x.push_back(d);
    // 关闭文件
    infile.close();

    // 打开文件
    infile.open(reference_lines_test_path + "/y");
    // 读取数据
    double dd;
    std::vector<double> sample_y;
    while (infile >> dd)
        sample_y.push_back(dd);
    // 关闭文件
    infile.close();

    for (size_t i = 0; i < sample_x.size(); i++)
    {
        geometry_msgs::PoseStamped pose_stamp;
        pose_stamp.header.frame_id = Frame_id;
        pose_stamp.header.stamp = ros::Time::now();
        pose_stamp.pose.position.x = sample_x[i];
        pose_stamp.pose.position.y = sample_y[i];
        pose_stamp.pose.position.z = 0;
        reference_lines.poses.push_back(pose_stamp);
    }
}

void MultilanePlanner::generate_reference_lines(std::string reference,
                                                std::vector<double> &accumulated_s,
                                                std::vector<ReferencePoint> &reference_points)
{
    std::string reference_lines_path = "";
    ros::param::get("referenceline_path", reference_lines_path);
    std::string reference_lines_test_path = reference_lines_path + "/" + reference;

    std::ifstream infile;
    // 打开文件
    infile.open(reference_lines_test_path + "/x");
    // 读取数据
    double d;
    std::vector<double> sample_x;
    while (infile >> d)
        sample_x.push_back(d);
    // 关闭文件
    infile.close();

    // 打开文件
    infile.open(reference_lines_test_path + "/y");
    // 读取数据
    double dd;
    std::vector<double> sample_y;
    while (infile >> dd)
        sample_y.push_back(dd);
    // 关闭文件
    infile.close();

    accumulated_s.clear();
    reference_points.clear();
    CubicSpline2D *csp;
    if (sample_x.size() > 0)
    {
        std::vector<double> headings;
        std::vector<double> kappas;
        std::vector<double> dkappas;
        std::vector<std::pair<double, double>> xy_points;

        for (size_t i = 0; i < sample_x.size(); i++)
        {
            std::pair<double, double> xy;
            xy.first = sample_x[i];
            xy.second = sample_y[i];
            xy_points.push_back(xy);
        }

        if (!PathMatcher::ComputePathProfile(xy_points, &headings, &accumulated_s, &kappas, &dkappas))
        {
            ROS_WARN("other rerferenceline generate failed!");
        }
        else
        {
            for (size_t i = 0; i < xy_points.size(); i++)
            {
                // 创建ReferencePoint类
                ReferencePoint reference_point(kappas[i], dkappas[i], xy_points[i].first, xy_points[i].second, headings[i],
                                               accumulated_s[i]);
                reference_points.emplace_back(reference_point);
            }
            ROS_WARN("other rerferenceline generate successfully!");
        }
    }
}

std::pair<double, double> MultilanePlanner::get_EgoLonForReference_other(const InitialConditions &planning_init_point,
                                                                         const std::vector<ReferencePoint> &discretized_ref_points)
{
    std::pair<double, double> ego_s_l;

    ego_s_l = PathMatcher::GetPathFrenetCoordinate(discretized_ref_points,
                                                   planning_init_point.x_init,
                                                   planning_init_point.y_init);
    return ego_s_l;
}

std::pair<double, double> MultilanePlanner::get_Stoppoint(const geometry_msgs::Pose &stop_point,
                                                          const std::vector<ReferencePoint> &discretized_ref_points)
{
    std::pair<double, double> stop_s_l;
    stop_s_l = PathMatcher::GetPathFrenetCoordinate(discretized_ref_points,
                                                    stop_point.position.x,
                                                    stop_point.position.y);
    return stop_s_l;
}

bool MultilanePlanner::CanChangeLaneToReference_other(std::shared_ptr<PathTimeGraph> &ptr_path_time_graph, double ego_s_other)
{
    bool can = true;
    //-------注意：只考虑两个障碍物AB的场景， B在前A在后  ---------//
    std::vector<double> s_lon;
    std::vector<double> v_ob;
    std::vector<double> lenght_ob;
    for (const auto &path_time_obstacle : ptr_path_time_graph->GetPathTimeObstacles()) // 遍历障碍物的ST信息
    {
        double s_lower = path_time_obstacle.bottom_left_point_.s();
        double s_higher = path_time_obstacle.upper_left_point_.s();
        s_lon.push_back(s_lower);
        s_lon.push_back(s_higher);
        v_ob.push_back(path_time_obstacle.obstacle_velocity);
        lenght_ob.push_back(path_time_obstacle.obstacle_length);
    }

    // 如果没有障碍物，可以变道
    if (s_lon.empty())
    {
        return true;
    }

    sort(s_lon.begin(), s_lon.end());
    double safe_a = Config_.FLAGS_vehicle_length + 2.0 * v_ob[1] + lenght_ob[1]; // 距离A车的安全距离
    double safe_b = Config_.FLAGS_vehicle_length + 3.0 * v_ob[0] + lenght_ob[0]; // 距离B车的安全距离

    // 判断自主车的位置
    if ((ego_s_other < s_lon.front()) && ((s_lon.front() - ego_s_other) < safe_b)) // 在B车后面，若能变道，从B车后面汇入
    {
        can = false;
    }
    else if ((ego_s_other > s_lon[2]) && (fabs(ego_s_other - s_lon[3]) < safe_a)) // 在A车前面，若能变道，从A车前面汇入
    {
        can = false;
    }
    else if ((ego_s_other > s_lon.front()) && (ego_s_other < s_lon[2]) && ((fabs(ego_s_other - s_lon[1]) < safe_b) && ((s_lon[2] - ego_s_other) < safe_a)))
    {
        can = false; // 在AB车中间，靠近B车
    }
    else if ((ego_s_other > s_lon.front()) && (ego_s_other < s_lon[2]) && ((fabs(ego_s_other - s_lon[1]) > safe_b) && ((s_lon[2] - ego_s_other) < safe_a)))
    {
        can = false; // 在AB车中间，靠近A车
    }

    return can;
}
