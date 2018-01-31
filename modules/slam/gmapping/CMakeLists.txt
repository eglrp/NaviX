cmake_minimum_required(VERSION 2.8.3)
project(gmapping)

include_directories(
    ${Boost_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${NaviX_SOURCE_DIR}/include/gmapping
)

add_subdirectory(gridfastslam)
add_subdirectory(scanmatcher)
add_subdirectory(sensor)
add_subdirectory(utils)
add_subdirectory(openslam)
