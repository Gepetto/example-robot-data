IF(NOT pinocchio_FOUND)
  MESSAGE(FATAL_ERROR "Pinocchio is required to build unit tests\n"
  "please either install Pinocchio or reconfigure without unit tests (-DBUILD_TESTING=OFF)")
ENDIF(NOT pinocchio_FOUND)

SET(${PROJECT_NAME}_PYTHON_TESTS
  load
  )

FOREACH(TEST ${${PROJECT_NAME}_PYTHON_TESTS})
  ADD_PYTHON_UNIT_TEST("py-${TEST}" "unittest/test_${TEST}.py" python)
  GET_TEST_PROPERTY("py-${TEST}" ENVIRONMENT env)
  SET_TESTS_PROPERTIES("py-${TEST}" PROPERTIES ENVIRONMENT
      "${env};ROS_PACKAGE_PATH=${CMAKE_SOURCE_DIR}/..:$ENV{ROS_PACKAGE_PATH}")
ENDFOREACH(TEST ${${PROJECT_NAME}_PYTHON_TESTS})
