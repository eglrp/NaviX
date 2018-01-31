cmake_minimum_required(VERSION 2.8.3)
project(test)

find_package(catkin REQUIRED
    COMPONENTS
        geometry_msgs
        sensor_msgs
        roscpp
        rospy
        std_srvs
        tf
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

# move_base
add_executable(test
  src/test.cpp
)
target_link_libraries(test
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
)
add_dependencies(test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
