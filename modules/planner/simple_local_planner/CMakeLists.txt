cmake_minimum_required(VERSION 2.8.3)
project(simple_local_planner)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED  COMPONENTS geometry_msgs tf cmake_modules costmap_2d nav_core nav_msgs pluginlib roscpp dynamic_reconfigure)

generate_dynamic_reconfigure_options(
  cfg/simple_local_planner.cfg
)

include_directories(${catkin_INCLUDE_DIRS})

add_library(simple_local_planner
    src/pure_pursuit.h
    src/pid.h
    src/simple_local_planner.cpp)

target_link_libraries(simple_local_planner ${catkin_LIBRARIES})

add_dependencies(simple_local_planner simple_local_planner_gencfg)
