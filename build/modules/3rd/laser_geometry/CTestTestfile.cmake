# CMake generated Testfile for 
# Source directory: /home/bailiqun/NaviX/modules/3rd/laser_geometry
# Build directory: /home/bailiqun/NaviX/build/modules/3rd/laser_geometry
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_laser_geometry_gtest_projection_test "/home/bailiqun/NaviX/build/catkin_generated/env_cached.sh" "/usr/bin/python" "/opt/ros/indigo/share/catkin/cmake/test/run_tests.py" "/home/bailiqun/NaviX/build/test_results/laser_geometry/gtest-projection_test.xml" "--return-code" "/home/bailiqun/NaviX/build/devel/lib/laser_geometry/projection_test --gtest_output=xml:/home/bailiqun/NaviX/build/test_results/laser_geometry/gtest-projection_test.xml")
add_test(_ctest_laser_geometry_nosetests_test.projection_test.py "/home/bailiqun/NaviX/build/catkin_generated/env_cached.sh" "/usr/bin/python" "/opt/ros/indigo/share/catkin/cmake/test/run_tests.py" "/home/bailiqun/NaviX/build/test_results/laser_geometry/nosetests-test.projection_test.py.xml" "--return-code" "/usr/bin/cmake -E make_directory /home/bailiqun/NaviX/build/test_results/laser_geometry" "/usr/bin/nosetests-2.7 -P --process-timeout=60 /home/bailiqun/NaviX/modules/3rd/laser_geometry/test/projection_test.py --with-xunit --xunit-file=/home/bailiqun/NaviX/build/test_results/laser_geometry/nosetests-test.projection_test.py.xml")
subdirs(../../../gtest)
