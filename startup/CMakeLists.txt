cmake_minimum_required(VERSION 2.8.3)
project(startup)

find_package(Boost REQUIRED COMPONENTS system thread signals)
find_package(catkin REQUIRED
  COMPONENTS
    tf
    rosbag_storage
    roscpp
    sensor_msgs
    slam
    joystick
    urg_node
    openslam
    serial
)

catkin_package(
    CATKIN_DEPENDS
        rosbag_storage
        geometry_msgs
        move_base_msgs
        nav_msgs
        roscpp
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ../include
    )

add_executable(navix src/startup.cpp)
target_link_libraries(navix
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  joystick
  urg_node_driver
  openslam
  serial
)

