cmake_minimum_required(VERSION 3.1)

# Project properties
set(PROJECT_ORG gepetto)
set(PROJECT_NAME example-robot-data)
set(PROJECT_DESCRIPTION
    "Set of robot URDFs for benchmarking and developed examples.")
set(PROJECT_URL https://github.com/${PROJECT_ORG}/${PROJECT_NAME})

# Project options
option(BUILD_PYTHON_INTERFACE "Build the python unit tests and helpers" ON)
option(INSTALL_PYTHON_INTERFACE_ONLY "Install *ONLY* the python interface" OFF)

# Project configuration
if(NOT INSTALL_PYTHON_INTERFACE_ONLY)
  set(PROJECT_USE_CMAKE_EXPORT TRUE)
endif(NOT INSTALL_PYTHON_INTERFACE_ONLY)
set(CUSTOM_HEADER_DIR ${PROJECT_NAME})
set(PROJECT_COMPATIBILITY_VERSION AnyNewerVersion)

# JRL-cmakemodule setup
include(cmake/base.cmake)

# Print initial message
message(STATUS "${PROJECT_DESCRIPTION}, version ${PROJECT_VERSION}")
message(STATUS "Copyright (C) 2018-2021 LAAS-CNRS, University of Edinburgh")
message(STATUS "All rights reserved.")
message(STATUS "Released under the BSD 3-Clause License.")

# Project definition
compute_project_args(PROJECT_ARGS LANGUAGES CXX)
project(${PROJECT_NAME} ${PROJECT_ARGS})

if(BUILD_PYTHON_INTERFACE)
  add_project_dependency(eigenpy 2.7.11 REQUIRED)
  add_project_dependency(pinocchio REQUIRED)
  string(REGEX REPLACE "-" "_" PY_NAME ${PROJECT_NAME})
  add_subdirectory(python)
  if(BUILD_TESTING)
    add_subdirectory(unittest)
  endif(BUILD_TESTING)
endif(BUILD_PYTHON_INTERFACE)

if(NOT INSTALL_PYTHON_INTERFACE_ONLY)
  add_library(${PROJECT_NAME} INTERFACE)
  target_include_directories(${PROJECT_NAME}
                             INTERFACE $<INSTALL_INTERFACE:include>)
  install(
    TARGETS ${PROJECT_NAME}
    EXPORT ${TARGETS_EXPORT_NAME}
    DESTINATION lib)

  add_subdirectory(include/${CUSTOM_HEADER_DIR})
  install(DIRECTORY robots DESTINATION share/${PROJECT_NAME})
  install(FILES package.xml DESTINATION share/${PROJECT_NAME})
endif(NOT INSTALL_PYTHON_INTERFACE_ONLY)
