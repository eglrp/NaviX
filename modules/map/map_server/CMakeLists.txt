cmake_minimum_required(VERSION 2.8.3)
project(map_server)

find_package(catkin REQUIRED
        COMPONENTS
            roscpp
            nav_msgs
            tf2
)

catkin_package(
    INCLUDE_DIRS
        ${NaviX_SOURCE_DIR}/include/
    LIBRARIES
        map_saver
    CATKIN_DEPENDS
        roscpp
        nav_msgs
        tf2
)

include_directories(
    ${NaviX_SOURCE_DIR}/include/
)

add_library(map_saver STATIC src/map_saver.cpp)
add_dependencies(map_saver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(map_saver
    ${catkin_LIBRARIES}
)


## Install executables and/or libraries
install(TARGETS map_saver
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)
