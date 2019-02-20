CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

INCLUDE(cmake/base.cmake)

SET(PROJECT_NAME example-robot-data)
SET(PROJECT_DESCRIPTION "Set of robot URDFs for benchmarking and developed examples.")

SET(PROJECT_URL https://gepgitlab.laas.fr/gepetto/${PROJECT_NAME})

SETUP_PROJECT()

INSTALL(DIRECTORY hyq DESTINATION share/${PROJECT_NAME})
INSTALL(DIRECTORY talos DESTINATION share/${PROJECT_NAME})

SETUP_PROJECT_FINALIZE()
