set(${PROJECT_NAME}_PYTHON_FILES
    human.py
    panda.py
    robots_loader.py
    talos.py
    utils.py
    __main__.py
    __init__.py)

foreach(python ${${PROJECT_NAME}_PYTHON_FILES})
  python_install_on_site(${PY_NAME} ${python})
endforeach()

get_property(
  EXAMPLE_ROBOT_DATA_MODEL_DIR_PROPERTY
  TARGET ${PROJECT_NAME}::${PROJECT_NAME}
  PROPERTY INTERFACE_COMPILE_DEFINITIONS)
string(REPLACE "EXAMPLE_ROBOT_DATA_MODEL_DIR=" ""
               EXAMPLE_ROBOT_DATA_MODEL_DIR_QUOTE
               ${EXAMPLE_ROBOT_DATA_MODEL_DIR_PROPERTY})
string(REPLACE "\"" "" EXAMPLE_ROBOT_DATA_MODEL_DIR
               ${EXAMPLE_ROBOT_DATA_MODEL_DIR_QUOTE})

configure_file(${PY_NAME}/path.py.in ${PY_NAME}/path.py)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${PY_NAME}/path.py
        DESTINATION "${PYTHON_SITELIB}/${PY_NAME}")
