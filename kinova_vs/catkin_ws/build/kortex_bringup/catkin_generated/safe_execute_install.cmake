execute_process(COMMAND "/home/user/vs/catkin_ws/build/kortex_bringup/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/user/vs/catkin_ws/build/kortex_bringup/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
