cmake_minimum_required(VERSION 2.8.3)
project(simple_local_planner)

find_package(catkin REQUIRED  COMPONENTS
    pcl_ros
    geometry_msgs
    tf
    cmake_modules
    nav_msgs
    pluginlib
    roscpp
    dynamic_reconfigure)

include_directories(
    ${Boost_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    include
    )

generate_dynamic_reconfigure_options(
  cfg/simple_local_planner.cfg
)

catkin_package(
    LIBRARIES simple_local_planner
    CATKIN_DEPENDS
        dynamic_reconfigure
        pluginlib
        roscpp
)

add_library(simple_local_planner src/simple_local_planner.cpp)
add_dependencies(simple_local_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(simple_local_planner ${catkin_LIBRARIES})

# Install targets
install(TARGETS simple_local_planner
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       )

install(FILES plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

