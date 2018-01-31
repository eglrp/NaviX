cmake_minimum_required(VERSION 2.8.3)
project(amcl)

find_package(catkin REQUIRED COMPONENTS)
catkin_package(CATKIN_DEPENDS)

find_package(Eigen3 REQUIRED)
add_definitions(${EIGEN3_DEFINITIONS})

include_directories(
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
