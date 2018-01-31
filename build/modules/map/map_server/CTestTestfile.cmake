# CMake generated Testfile for 
# Source directory: /home/bailiqun/NaviX/modules/map/map_server
# Build directory: /home/bailiqun/NaviX/build/modules/map/map_server
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_map_server_gtest_map_server_utest "/home/bailiqun/NaviX/build/catkin_generated/env_cached.sh" "/usr/bin/python" "/opt/ros/indigo/share/catkin/cmake/test/run_tests.py" "/home/bailiqun/NaviX/build/test_results/map_server/gtest-map_server_utest.xml" "--return-code" "/home/bailiqun/NaviX/build/devel/lib/map_server/map_server_utest --gtest_output=xml:/home/bailiqun/NaviX/build/test_results/map_server/gtest-map_server_utest.xml")
add_test(_ctest_map_server_rostest_test_rtest.xml "/home/bailiqun/NaviX/build/catkin_generated/env_cached.sh" "/usr/bin/python" "/opt/ros/indigo/share/catkin/cmake/test/run_tests.py" "/home/bailiqun/NaviX/build/test_results/map_server/rostest-test_rtest.xml" "--return-code" "/opt/ros/indigo/share/rostest/cmake/../../../bin/rostest --pkgdir=/home/bailiqun/NaviX/modules/map/map_server --package=map_server --results-filename test_rtest.xml --results-base-dir \"/home/bailiqun/NaviX/build/test_results\" /home/bailiqun/NaviX/modules/map/map_server/test/rtest.xml ")
