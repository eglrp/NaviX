# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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
CMAKE_SOURCE_DIR = /home/bailiqun/urg_node

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/bailiqun/urg_node/build

# Utility rule file for clean_test_results.

# Include the progress variables for this target.
include urg_c/CMakeFiles/clean_test_results.dir/progress.make

urg_c/CMakeFiles/clean_test_results:
	cd /home/bailiqun/urg_node/build/urg_c && /usr/bin/python /opt/ros/indigo/share/catkin/cmake/test/remove_test_results.py /home/bailiqun/urg_node/build/test_results

clean_test_results: urg_c/CMakeFiles/clean_test_results
clean_test_results: urg_c/CMakeFiles/clean_test_results.dir/build.make
.PHONY : clean_test_results

# Rule to build all files generated by this target.
urg_c/CMakeFiles/clean_test_results.dir/build: clean_test_results
.PHONY : urg_c/CMakeFiles/clean_test_results.dir/build

urg_c/CMakeFiles/clean_test_results.dir/clean:
	cd /home/bailiqun/urg_node/build/urg_c && $(CMAKE_COMMAND) -P CMakeFiles/clean_test_results.dir/cmake_clean.cmake
.PHONY : urg_c/CMakeFiles/clean_test_results.dir/clean

urg_c/CMakeFiles/clean_test_results.dir/depend:
	cd /home/bailiqun/urg_node/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/bailiqun/urg_node /home/bailiqun/urg_node/urg_c /home/bailiqun/urg_node/build /home/bailiqun/urg_node/build/urg_c /home/bailiqun/urg_node/build/urg_c/CMakeFiles/clean_test_results.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : urg_c/CMakeFiles/clean_test_results.dir/depend

