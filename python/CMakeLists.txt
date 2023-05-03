set(${PROJECT_NAME}_PYTHON_FILES robots_loader.py __main__.py __init__.py)

foreach(python ${${PROJECT_NAME}_PYTHON_FILES})
  python_install_on_site(${PY_NAME} ${python})
endforeach(python ${${PROJECT_NAME}_PYTHON_FILES})

configure_file(${PY_NAME}/path.py.in ${PY_NAME}/path.py)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${PY_NAME}/path.py
        DESTINATION "${PYTHON_SITELIB}/${PY_NAME}")
