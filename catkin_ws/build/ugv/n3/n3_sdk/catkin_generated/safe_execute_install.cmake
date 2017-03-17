execute_process(COMMAND "/home/ubuntu/fyp/catkin_ws/build/ugv/n3/n3_sdk/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/ubuntu/fyp/catkin_ws/build/ugv/n3/n3_sdk/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
