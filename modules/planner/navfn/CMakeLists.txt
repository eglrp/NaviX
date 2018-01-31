cmake_minimum_required(VERSION 2.8)
project(navfn)

find_package(catkin REQUIRED COMPONENTS
    cmake_modules
    costmap_2d
    geometry_msgs
    message_generation
    nav_core
    nav_msgs
    pcl_conversions
    pcl_ros
    pluginlib
    rosconsole
    roscpp
    tf
    visualization_msgs
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

include_directories(
    include
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)
add_definitions(${EIGEN3_DEFINITIONS})

add_library (navfn src/navfn.cpp src/navfn_ros.cpp)
target_link_libraries(navfn ${catkin_LIBRARIES})
