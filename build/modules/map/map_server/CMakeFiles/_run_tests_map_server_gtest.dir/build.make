# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/bailiqun/NaviX

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/bailiqun/NaviX/build

# Utility rule file for _run_tests_map_server_gtest.

# Include the progress variables for this target.
include modules/map/map_server/CMakeFiles/_run_tests_map_server_gtest.dir/progress.make

_run_tests_map_server_gtest: modules/map/map_server/CMakeFiles/_run_tests_map_server_gtest.dir/build.make

.PHONY : _run_tests_map_server_gtest

# Rule to build all files generated by this target.
modules/map/map_server/CMakeFiles/_run_tests_map_server_gtest.dir/build: _run_tests_map_server_gtest

.PHONY : modules/map/map_server/CMakeFiles/_run_tests_map_server_gtest.dir/build

modules/map/map_server/CMakeFiles/_run_tests_map_server_gtest.dir/clean:
	cd /home/bailiqun/NaviX/build/modules/map/map_server && $(CMAKE_COMMAND) -P CMakeFiles/_run_tests_map_server_gtest.dir/cmake_clean.cmake
.PHONY : modules/map/map_server/CMakeFiles/_run_tests_map_server_gtest.dir/clean

modules/map/map_server/CMakeFiles/_run_tests_map_server_gtest.dir/depend:
	cd /home/bailiqun/NaviX/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/bailiqun/NaviX /home/bailiqun/NaviX/modules/map/map_server /home/bailiqun/NaviX/build /home/bailiqun/NaviX/build/modules/map/map_server /home/bailiqun/NaviX/build/modules/map/map_server/CMakeFiles/_run_tests_map_server_gtest.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : modules/map/map_server/CMakeFiles/_run_tests_map_server_gtest.dir/depend

