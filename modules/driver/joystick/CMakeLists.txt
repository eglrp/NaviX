cmake_minimum_required(VERSION 2.8.3)
project(joystick)

find_package(catkin REQUIRED roscpp diagnostic_updater sensor_msgs)
catkin_package(DEPENDS roscpp diagnostic_updater sensor_msgs)

include_directories(
    include
)

add_library(joystick STATIC src/joy.cpp)
target_link_libraries(joystick ${catkin_LIBRARIES})

# Install targets
install(TARGETS joystick
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY migration_rules
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
