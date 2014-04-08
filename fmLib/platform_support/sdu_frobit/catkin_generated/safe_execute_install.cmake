execute_process(COMMAND "/home/morten/roswork/sdu/src/fmLib/platform_support/sdu_frobit/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/morten/roswork/sdu/src/fmLib/platform_support/sdu_frobit/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
