cmake_minimum_required(VERSION 2.8.3)
project(rotate_recovery)

find_package(catkin REQUIRED
        COMPONENTS
            cmake_modules
            roscpp
            tf
            pluginlib
        )

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

include_directories(
    ${NaviX_SOURCE_DIR}/include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)


add_definitions(${EIGEN3_DEFINITIONS})

catkin_package(
    INCLUDE_DIRS ${NaviX_SOURCE_DIR}/include
    LIBRARIES rotate_recovery
    CATKIN_DEPENDS
        roscpp
        pluginlib
)

add_library(rotate_recovery src/rotate_recovery.cpp)
add_dependencies(rotate_recovery ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rotate_recovery ${catkin_LIBRARIES})

install(TARGETS rotate_recovery
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(FILES rotate_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

