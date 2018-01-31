cmake_minimum_required(VERSION 2.8)
project(slam)

include_directories(
    include
    ${Boost_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS})

INCLUDE_DIRECTORIES(include)

add_subdirectory(amcl)
add_subdirectory(gmapping)

#find_package(catkin REQUIRED  roscpp tf rosbag_storage)
#find_package(Boost REQUIRED signals)

#catkin_package(
#    INCLUDE_DIRS
#        include
#    LIBRARIES amcl openslam
#)

#add_executable(mapping main.cpp)
#target_link_libraries(mapping
#    ${Boost_LIBRARIES}
#    ${catkin_LIBRARIES}
#    openslam
#    amcl)

#install(DIRECTORY include
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  PATTERN ".svn" EXCLUDE
#)
