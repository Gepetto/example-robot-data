CMAKE_MINIMUM_REQUIRED(VERSION 3.0)

# Set up project properties
SET(PROJECT_NAMESPACE gepetto)
SET(PROJECT_NAME example-robot-data)
SET(PROJECT_DESCRIPTION "Set of robot URDFs for benchmarking and developed examples.")
SET(PROJECT_URL https://gepgitlab.laas.fr/${PROJECT_NAMESPACE}/${PROJECT_NAME})

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/test.cmake)
INCLUDE(cmake/python.cmake)

# Print initial message
MESSAGE("${PROJECT_DESCRIPTION}, version ${PROJECT_VERSION}")
MESSAGE("Copyright (C) 2018-2020 CNRS-LAAS / University of Edinburgh")
MESSAGE("All rights reserved.")
MESSAGE("Released under the BSD 3-Clause License.")

# Set a default build type to 'Release' if none was specified
IF(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  MESSAGE(STATUS "Setting build type to 'Release' as none was specified.")
  SET(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  SET_PROPERTY(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
ENDIF()

SET(CUSTOM_HEADER_DIR ${PROJECT_NAME})

CMAKE_POLICY(SET CMP0048 NEW)
PROJECT(${PROJECT_NAME} VERSION ${PROJECT_VERSION_FULL} LANGUAGES CXX)

OPTION (BUILD_PYTHON_INTERFACE "Build the python unit tests and helpers" ON)
OPTION (INSTALL_PYTHON_INTERFACE_ONLY "Install *ONLY* the python interface" OFF)

IF(BUILD_PYTHON_INTERFACE)
  ADD_REQUIRED_DEPENDENCY("pinocchio >= 2.1.0")
  FINDPYTHON()
  STRING(REGEX REPLACE "-" "_" PY_NAME ${PROJECT_NAME})
  ADD_SUBDIRECTORY(python)
  ADD_SUBDIRECTORY(unittest)
ENDIF(BUILD_PYTHON_INTERFACE)

IF(NOT INSTALL_PYTHON_INTERFACE_ONLY)
  ADD_SUBDIRECTORY(include/${CUSTOM_HEADER_DIR})
  INSTALL(DIRECTORY robots DESTINATION share/${PROJECT_NAME})
  INSTALL(FILES package.xml DESTINATION share/${PROJECT_NAME})
ENDIF(NOT INSTALL_PYTHON_INTERFACE_ONLY)
