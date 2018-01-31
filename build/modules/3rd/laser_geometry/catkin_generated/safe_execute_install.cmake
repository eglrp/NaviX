execute_process(COMMAND "/home/bailiqun/NaviX/build/modules/3rd/laser_geometry/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/bailiqun/NaviX/build/modules/3rd/laser_geometry/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
