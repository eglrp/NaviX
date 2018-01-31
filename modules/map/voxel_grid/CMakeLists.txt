cmake_minimum_required(VERSION 2.8.3)
project(voxel_grid)

find_package(catkin REQUIRED COMPONENTS
    roscpp
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(voxel_grid src/voxel_grid.cpp)
add_dependencies(voxel_grid ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(voxel_grid ${catkin_LIBRARIES})
