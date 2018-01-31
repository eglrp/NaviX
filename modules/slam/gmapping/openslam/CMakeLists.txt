cmake_minimum_required(VERSION 2.8.3)
project(openslam)

find_package(catkin REQUIRED COMPONENTS rosbag_storage)
catkin_package(CATKIN_DEPENDS rosbag_storage)

INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}
    ${catkin_INCLUDE_DIRS}
)

add_library(openslam slam_gmapping.cpp)
target_link_libraries(openslam
    gridfastslam
    scanmatcher
    sensor_base
    sensor_odometry
    sensor_range
    utils
    ${catkin_LIBRARIES})

install(
    TARGETS
        openslam
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
