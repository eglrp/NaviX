cmake_minimum_required(VERSION 2.8.12)
project(serial)

find_package(catkin REQUIRED COMPONENTS  roscpp sensor_msgs nav_msgs geometry_msgs tf)
find_package(Boost REQUIRED COMPONENTS system thread)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
    ${NaviX_SOURCE_DIR}/include
)

catkin_package(
  INCLUDE_DIRS ${SERIAL_INCLUDE}
  LIBRARIES serial
  CATKIN_DEPENDS roscpp sensor_msgs nav_msgs geometry_msgs tf
  DEPENDS
)

link_directories(${Boost_LIBRARY_DIRS})
add_library(serial STATIC
    serial.cpp
)
target_link_libraries(serial
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
)

# Install targets
install(TARGETS serial
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
