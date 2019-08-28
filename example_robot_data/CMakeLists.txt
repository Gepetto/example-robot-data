SET(${PROJECT_NAME}_PYTHON_FILES
  robots_loader.py
  display.py
  __init__.py
  )

FOREACH(python ${${PROJECT_NAME}_PYTHON_FILES})
  PYTHON_BUILD(. ${python})
  INSTALL(FILES ${python} DESTINATION ${PYTHON_SITELIB}/${PY_NAME})
ENDFOREACH(python ${${PROJECT_NAME}_PYTHON_FILES})
