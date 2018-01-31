cmake_minimum_required(VERSION 2.8.3)
project(amcl)

find_package(Boost REQUIRED)
find_package(catkin REQUIRED
        COMPONENTS
            message_filters
            rosbag
            roscpp
            std_srvs
            tf
            dynamic_reconfigure
            nav_msgs
            std_srvs)

# dynamic reconfigure
generate_dynamic_reconfigure_options(
    cfg/AMCL.cfg
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS
        rosbag
        roscpp
        dynamic_reconfigure
        tf
  CATKIN_DEPENDS nav_msgs std_srvs
  LIBRARIES amcl_sensors amcl_map amcl_pf
)

include_directories(
    ${NaviX_SOURCE_DIR}/include
    ${Boost_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

add_subdirectory(map)
add_subdirectory(pf)
add_subdirectory(sensors)

add_library(amcl STATIC amcl.cpp)
target_link_libraries(amcl
    amcl_map
    amcl_pf
    amcl_sensors)

## Mark cpp header files for installation
install(
    TARGETS amcl
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
