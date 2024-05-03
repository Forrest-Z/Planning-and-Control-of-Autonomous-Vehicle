cmake_minimum_required(VERSION 3.0.2)
project(global_routing)

## Compile as C++11, supported in ROS Kinetic,melodic
##Compile as C++14, supported in ROS nortic
add_compile_options(-std=c++14)


file(GLOB_RECURSE SRC_FILE     
  ${PROJECT_SOURCE_DIR}/src/*.cpp 
)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
)
find_package(Eigen3 QUIET)
find_package(OsqpEigen QUIET)

catkin_package(
  INCLUDE_DIRS include
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  include/referenceline
  include/control
  include/config
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(global_routing_node ${SRC_FILE})
target_link_libraries(global_routing_node ${catkin_LIBRARIES} osqp adolc ipopt)
