CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/test.cmake)
INCLUDE(cmake/python.cmake)

# Set up project properties
SET(PROJECT_NAMESPACE gepetto)
SET(PROJECT_NAME example-robot-data)
SET(PROJECT_DESCRIPTION "Set of robot URDFs for benchmarking and developed examples.")
SET(PROJECT_URL https://gepgitlab.laas.fr/${PROJECT_NAMESPACE}/${PROJECT_NAME})

# Print initial message
MESSAGE("${PROJECT_DESCRIPTION}, version ${PROJECT_VERSION}")
MESSAGE("Copyright (C) 2018-2019 CNRS-LAAS")
MESSAGE("All rights reserved.")
MESSAGE("Released under the BSD 3-Clause License.")

SETUP_PROJECT()

OPTION (INSTALL_DATA "Install the data" ON)
OPTION (BUILD_PYTHON_INTERFACE "Build the python unit tests and helpers" ON)

IF(BUILD_PYTHON_INTERFACE)
    ADD_REQUIRED_DEPENDENCY("pinocchio >= 2.1.0")
    FINDPYTHON()
    STRING(REGEX REPLACE "-" "_" PY_NAME ${PROJECT_NAME})
    ADD_SUBDIRECTORY(${PY_NAME})
    ADD_SUBDIRECTORY(unittest)
ENDIF(BUILD_PYTHON_INTERFACE)

IF(INSTALL_DATA)
    INSTALL(DIRECTORY hyq_description DESTINATION share/${PROJECT_NAME})
    INSTALL(DIRECTORY icub_description DESTINATION share/${PROJECT_NAME})
    INSTALL(DIRECTORY talos_data DESTINATION share/${PROJECT_NAME})
    INSTALL(DIRECTORY tiago_description DESTINATION share/${PROJECT_NAME})
ENDIF(INSTALL_DATA)

SETUP_PROJECT_FINALIZE()
