cmake_minimum_required(VERSION 2.8.3)
project(clear_costmap_recovery)

find_package(catkin REQUIRED
        COMPONENTS
            cmake_modules
            pluginlib
            roscpp
            tf)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

include_directories(
    ${NaviX_SOURCE_DIR}/include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

add_definitions(${EIGEN3_DEFINITIONS})

catkin_package(
    INCLUDE_DIRS ${NaviX_SOURCE_DIR}/include
    LIBRARIES clear_costmap_recovery
    CATKIN_DEPENDS
        pluginlib
        roscpp
        tf
)

add_library(clear_costmap_recovery src/clear_costmap_recovery.cpp)
add_dependencies(clear_costmap_recovery ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(clear_costmap_recovery ${catkin_LIBRARIES})


## Configure Tests
if(CATKIN_ENABLE_TESTING)
  # Find package test dependencies
  find_package(rostest REQUIRED)

  # Add the test folder to the include directories
  include_directories(test)

  # Create targets for test executables
  add_rostest_gtest(clear_tester test/clear_tests.launch test/clear_tester.cpp)
  target_link_libraries(clear_tester clear_costmap_recovery ${GTEST_LIBRARIES})
endif()


install(TARGETS clear_costmap_recovery
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       )

install(FILES ccr_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
