if(NOT pinocchio_FOUND)
  message(
    FATAL_ERROR
      "Pinocchio is required to build unit tests\n"
      "please either install Pinocchio or reconfigure without unit tests (-DBUILD_TESTING=OFF)"
  )
endif(NOT pinocchio_FOUND)

set(${PROJECT_NAME}_PYTHON_TESTS load)

foreach(TEST ${${PROJECT_NAME}_PYTHON_TESTS})
  add_python_unit_test("py-${TEST}" "unittest/test_${TEST}.py" python)
endforeach(TEST ${${PROJECT_NAME}_PYTHON_TESTS})
