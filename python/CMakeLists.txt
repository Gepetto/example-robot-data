SET(${PROJECT_NAME}_PYTHON_FILES
  robots_loader.py
  __main__.py
  __init__.py
  )

FOREACH(python ${${PROJECT_NAME}_PYTHON_FILES})
  PYTHON_INSTALL_ON_SITE(${PY_NAME} ${python})
ENDFOREACH(python ${${PROJECT_NAME}_PYTHON_FILES})
