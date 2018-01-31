cmake_minimum_required(VERSION 2.8.3)
project(urg_c)

# Set library locations to better handle if upstream locations change
set(LIBRARY_DIR current)
set(LIBRARY_INCLUDE_DIR ${LIBRARY_DIR}/include)
set(LIBRARY_SRC_DIR ${LIBRARY_DIR}/src)
set(LIBRARY_SAMPLE_DIR ${LIBRARY_DIR}/samples)

find_package(catkin REQUIRED)

catkin_package(
  INCLUDE_DIRS ${LIBRARY_INCLUDE_DIR}
  LIBRARIES liburg_c
  CATKIN_DEPENDS
  DEPENDS
)

## Specify additional locations of header files
include_directories(${LIBRARY_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

## Declare libraries
add_library(liburg_c ${LIBRARY_SRC_DIR}/urg_sensor.c
                         ${LIBRARY_SRC_DIR}/urg_utils.c
                         ${LIBRARY_SRC_DIR}/urg_debug.c
                         ${LIBRARY_SRC_DIR}/urg_connection.c
                         ${LIBRARY_SRC_DIR}/urg_ring_buffer.c
                         ${LIBRARY_SRC_DIR}/urg_serial.c
                         ${LIBRARY_SRC_DIR}/urg_serial_utils.c
                         ${LIBRARY_SRC_DIR}/urg_tcpclient.c
                         ${LIBRARY_SRC_DIR}/urg_time.c)
target_link_libraries(liburg_c -lrt -lm)

add_library(open_urg_sensor ${LIBRARY_SAMPLE_DIR}/open_urg_sensor.c)
target_link_libraries(open_urg_sensor -lm liburg_c ${catkin_LIBRARIES})

## Mark executables and/or libraries for installation
install(TARGETS liburg_c open_urg_sensor
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY ${LIBRARY_INCLUDE_DIR}/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
