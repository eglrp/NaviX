cmake_minimum_required(VERSION 2.8.3)
project(urg_node)

add_subdirectory(urg_c)

find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure laser_proc message_generation roscpp sensor_msgs std_srvs tf)
set_directory_properties(PROPERTIES COMPILE_OPTIONS "-std=c++11")

include_directories(
    ${NaviX_SOURCE_DIR}/include
    ${catkin_INCLUDE_DIRS})

# Dynamic reconfigure support
generate_dynamic_reconfigure_options(cfg/URG.cfg)

add_message_files(FILES
  Status.msg
)

generate_messages(
  DEPENDENCIES std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES urg_c_wrapper urg_node_driver
  CATKIN_DEPENDS dynamic_reconfigure laser_proc message_runtime roscpp sensor_msgs std_msgs std_srvs
  DEPENDS
)

## Declare a cpp library
add_library(urg_c_wrapper src/urg_c_wrapper.cpp)
target_link_libraries(urg_c_wrapper
    ${catkin_LIBRARIES}
    liburg_c
    open_urg_sensor
)

# Declare the cpp as a lib also
add_library(urg_node_driver src/urg_node_driver.cpp)
add_dependencies(urg_node_driver ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_gencpp)
target_link_libraries(urg_node_driver urg_c_wrapper ${catkin_LIBRARIES})

#### Declare a cpp executable
add_executable(urg_node src/urg_node.cpp)
target_link_libraries(urg_node urg_node_driver ${catkin_LIBRARIES})

### Declare getID tool
add_executable(getID src/getID.cpp)
target_link_libraries(getID urg_c_wrapper ${catkin_LIBRARIES})

## Mark executables and/or libraries for installation
install(TARGETS urg_node urg_node_driver urg_c_wrapper
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

