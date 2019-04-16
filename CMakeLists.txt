CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/test.cmake)
INCLUDE(cmake/python.cmake)

SET(PROJECT_NAMESPACE gepetto)
SET(PROJECT_NAME example-robot-data)
SET(PROJECT_DESCRIPTION "Set of robot URDFs for benchmarking and developed examples.")

SET(PROJECT_URL https://gepgitlab.laas.fr/${PROJECT_NAMESPACE}/${PROJECT_NAME})

SETUP_PROJECT()

ADD_REQUIRED_DEPENDENCY("pinocchio >= 2.1.0")

FINDPYTHON()

INSTALL(DIRECTORY hyq_description DESTINATION share/${PROJECT_NAME})
INSTALL(DIRECTORY talos_data DESTINATION share/${PROJECT_NAME})

ADD_SUBDIRECTORY(python)
ADD_SUBDIRECTORY(unittest)

SETUP_PROJECT_FINALIZE()
