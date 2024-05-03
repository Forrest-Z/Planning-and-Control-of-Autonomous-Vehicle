#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

visualization_msgs::Marker car_marker;
ros::Publisher car_publish;
double speed, steer;
double wheel_base = 2.7; // 轴距（前后轮之车轮轴距离）
double start_x, start_y, start_theta;

void visualization_car(const double x, const double y, const double th)
{
    double Length = 4.4; // 车长
    double width = 2.0;  // 车宽
    car_marker.pose.orientation = tf::createQuaternionMsgFromYaw(th);
    car_marker.header.frame_id = "map";
    car_marker.header.stamp = ros::Time::now();
    car_marker.ns = "basic_shapes";
    car_marker.id = 0; // 注意了
    car_marker.type = visualization_msgs::Marker::ARROW;
    car_marker.action = visualization_msgs::Marker::ADD;
    car_marker.pose.position.x = x;
    car_marker.pose.position.y = y;
    car_marker.pose.position.z = 0;
    car_marker.scale.x = Length;
    car_marker.scale.y = width;
    car_marker.scale.z = 1.0;
    car_marker.color.r = 1.0f;
    car_marker.color.g = 1.0f;
    car_marker.color.b = 0.0f;
    car_marker.color.a = 1.0;
    car_marker.lifetime = ros::Duration();
    car_publish.publish(car_marker);
}

void control_data(const geometry_msgs::Vector3 &v)
{
    speed = v.x;
    steer = v.y;
}

void car_start(const geometry_msgs::Vector3 &msg)
{
    start_x = msg.x;
    start_y = msg.y;
    start_theta = msg.z;
    speed = 0;
    steer = start_theta;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "car_simulation");
    ros::NodeHandle n;
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/xsj/odom", 100);
    car_publish = n.advertise<visualization_msgs::Marker>("/xsj/car/car_state", 100);
    ros::Subscriber sub = n.subscribe("/xsj/car/control_car", 100, control_data);           // 订阅控制信息
    ros::Subscriber car_start_pose = n.subscribe("/xsj/car/car_start_pose", 10, car_start); // 订阅起点信息
    ros::Time cur_time, last_time;
    cur_time = ros::Time::now();
    last_time = cur_time;
    ros::Rate rate(10);

    while (n.ok())
    {
        cur_time = ros::Time::now();
        
        visualization_car(start_x, start_y, start_theta);

        double dt = (cur_time - last_time).toSec();
        double delta_x = speed * cos(start_theta) * dt;
        double delta_y = speed * sin(start_theta) * dt;
        double delta_theta = (speed * tan(steer) / wheel_base) * dt;

        start_x += delta_x;
        start_y += delta_y;
        start_theta += delta_theta;

        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(start_theta);
        // 填充里程计数据
        nav_msgs::Odometry odom;
        odom.header.frame_id = "map";
        odom.header.stamp = cur_time;
        // 位置信息
        odom.pose.pose.position.x = start_x;
        odom.pose.pose.position.y = start_y;
        odom.pose.pose.orientation = odom_quat;
        // 发布里程计
        odom_pub.publish(odom);
        last_time = cur_time;

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}