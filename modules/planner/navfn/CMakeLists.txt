cmake_minimum_required(VERSION 2.8.3)
project(navfn)

find_package(catkin REQUIRED
    COMPONENTS
        cmake_modules
        costmap_2d
        geometry_msgs
        message_generation
        nav_core
        nav_msgs
        pcl_conversions
        pcl_ros
        pluginlib
        rosconsole
        roscpp
        tf
        visualization_msgs
        )

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
remove_definitions(-DDISABLE_LIBUSB-1.0)
include_directories(
    ${NaviX_SOURCE_DIR}/include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)
add_definitions(${EIGEN3_DEFINITIONS})

# services
add_service_files(
    DIRECTORY srv
    FILES
    MakeNavPlan.srv
    SetCostmap.srv
)

generate_messages(
    DEPENDENCIES
        geometry_msgs
)

catkin_package(
    INCLUDE_DIRS
        ${NaviX_SOURCE_DIR}/include
    LIBRARIES
        navfn
    CATKIN_DEPENDS
        geometry_msgs
        message_runtime
        nav_core
        nav_msgs
        pluginlib
        roscpp
        visualization_msgs
)

add_library (navfn src/navfn.cpp src/navfn_ros.cpp)
target_link_libraries(navfn
    ${catkin_LIBRARIES}
    )
add_dependencies(navfn ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

add_executable(navfn_node src/navfn_node.cpp)
target_link_libraries(navfn_node
    navfn
    )

install(TARGETS navfn navfn_node
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
       )

install(DIRECTORY include/navfn/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES bgp_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

