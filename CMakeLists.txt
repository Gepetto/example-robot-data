cmake_minimum_required(VERSION 3.10)

# Project properties
set(PROJECT_ORG gepetto)
set(PROJECT_NAME example-robot-data)
set(PROJECT_DESCRIPTION
    "Set of robot URDFs for benchmarking and developed examples")
set(PROJECT_URL https://github.com/${PROJECT_ORG}/${PROJECT_NAME})
# To enable jrl-cmakemodules compatibility with workspace we must define the two
# following lines
set(PROJECT_AUTO_RUN_FINALIZE FALSE)
set(PROJECT_SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR})

# Project options
option(BUILD_PYTHON_INTERFACE "Build the python unit tests and helpers" ON)
option(INSTALL_PYTHON_INTERFACE_ONLY "Install *ONLY* the python interface" OFF)

# Project configuration
if(NOT INSTALL_PYTHON_INTERFACE_ONLY)
  set(PROJECT_USE_CMAKE_EXPORT TRUE)
endif(NOT INSTALL_PYTHON_INTERFACE_ONLY)
set(CUSTOM_HEADER_DIR ${PROJECT_NAME})
set(PROJECT_COMPATIBILITY_VERSION AnyNewerVersion)

# Check if the submodule cmake have been initialized
set(JRL_CMAKE_MODULES "${CMAKE_CURRENT_LIST_DIR}/cmake")
if(EXISTS "${JRL_CMAKE_MODULES}/base.cmake")
  message(STATUS "JRL cmakemodules found in 'cmake/' git submodule")
else()
  find_package(jrl-cmakemodules QUIET CONFIG)
  if(jrl-cmakemodules_FOUND)
    get_property(
      JRL_CMAKE_MODULES
      TARGET jrl-cmakemodules::jrl-cmakemodules
      PROPERTY INTERFACE_INCLUDE_DIRECTORIES)
    message(STATUS "JRL cmakemodules found on system at ${JRL_CMAKE_MODULES}")
  elseif(${CMAKE_VERSION} VERSION_LESS "3.14.0")
    message(
      FATAL_ERROR
        "\nCan't find jrl-cmakemodules. Please either:\n"
        "  - use git submodule: 'git submodule update --init'\n"
        "  - or install https://github.com/jrl-umi3218/jrl-cmakemodules\n"
        "  - or upgrade your CMake version to >= 3.14 to allow automatic fetching\n"
    )
  else()
    message(STATUS "JRL cmakemodules not found. Let's fetch it.")
    include(FetchContent)
    FetchContent_Declare(
      "jrl-cmakemodules"
      GIT_REPOSITORY "https://github.com/jrl-umi3218/jrl-cmakemodules.git")
    FetchContent_MakeAvailable("jrl-cmakemodules")
    FetchContent_GetProperties("jrl-cmakemodules" SOURCE_DIR JRL_CMAKE_MODULES)
  endif()
endif()

# JRL-cmakemodule setup
include("${JRL_CMAKE_MODULES}/base.cmake")
include("${JRL_CMAKE_MODULES}/python.cmake")

# Print initial message
message(STATUS "${PROJECT_DESCRIPTION}, version ${PROJECT_VERSION}")
message(STATUS "Copyright (C) 2018-2023 LAAS-CNRS, University of Edinburgh")
message(STATUS "                        Heriot-Watt University, INRIA")
message(STATUS "All rights reserved.")
message(STATUS "Released under the BSD 3-Clause License.")

# Project definition
compute_project_args(PROJECT_ARGS LANGUAGES CXX)
project(${PROJECT_NAME} ${PROJECT_ARGS})

if(BUILD_PYTHON_INTERFACE)
  add_project_dependency(eigenpy 3.0.0 REQUIRED)
  add_project_dependency(pinocchio 2.7.0 REQUIRED)
  string(REGEX REPLACE "-" "_" PY_NAME ${PROJECT_NAME})
  findpython(REQUIRED)
  add_subdirectory(python)
  if(BUILD_TESTING)
    add_subdirectory(unittest)
  endif(BUILD_TESTING)
endif()

if(NOT INSTALL_PYTHON_INTERFACE_ONLY)
  add_library(${PROJECT_NAME} INTERFACE)
  add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
  target_include_directories(
    ${PROJECT_NAME}
    INTERFACE $<INSTALL_INTERFACE:include>
              $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
              $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  target_compile_definitions(
    ${PROJECT_NAME}
    INTERFACE
      $<INSTALL_INTERFACE:EXAMPLE_ROBOT_DATA_MODEL_DIR="$<INSTALL_PREFIX>/share/${PROJECT_NAME}/robots">
      $<BUILD_INTERFACE:EXAMPLE_ROBOT_DATA_MODEL_DIR="${CMAKE_CURRENT_SOURCE_DIR}/robots">
  )
  install(
    TARGETS ${PROJECT_NAME}
    EXPORT ${TARGETS_EXPORT_NAME}
    DESTINATION lib)

  install(FILES include/${CUSTOM_HEADER_DIR}/path.hpp
          DESTINATION include/${CUSTOM_HEADER_DIR})
  install(DIRECTORY robots DESTINATION share/${PROJECT_NAME})
  install(FILES package.xml DESTINATION share/${PROJECT_NAME})
endif()

setup_project_finalize()
