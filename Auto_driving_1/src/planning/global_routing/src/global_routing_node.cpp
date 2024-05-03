#include "global_routing.h"
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "global_routing");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);

  GlobalRouting G_global_routing;
  while (ros::ok())
  {
  }
  return 0;
}