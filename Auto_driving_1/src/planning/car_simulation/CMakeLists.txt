cmake_minimum_required(VERSION 2.8.3)
project(car_simulation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
)

catkin_package(
    INCLUDE_DIRS include
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)


set(SRC_FILE
  src/car_state.cpp
)

add_executable(car_model_node ${SRC_FILE})
target_link_libraries(car_model_node ${catkin_LIBRARIES})
