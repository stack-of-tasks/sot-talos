# Copyright 2016, Thomas Moulard, Olivier Stasse, JRL, CNRS/AIST
#

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(PROJECT_NAMESPACE stack-of-tasks)
SET(PROJECT_NAME sot-talos)
SET(PROJECT_DESCRIPTION "dynamic-graph package for Talos robot")
SET(PROJECT_URL "http://github.com/${PROJECT_NAMESPACE}/${PROJECT_NAME}")

# Export CMake Target
SET(PROJECT_USE_CMAKE_EXPORT TRUE)

# Disable -Werror on Unix for now.
SET(CXX_DISABLE_WERROR True)

INCLUDE(cmake/base.cmake)

# Specify the project.
cmake_policy(SET CMP0048 NEW)
PROJECT(${PROJECT_NAME}
  LANGUAGES
  CXX
  VERSION
  ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}.${PROJECT_VERSION_PATCH}
  )

find_package (Boost REQUIRED
  python filesystem system thread program_options unit_test_framework)
find_package (Eigen3 REQUIRED NO_MODULE)

INCLUDE(cmake/lapack.cmake)
INCLUDE(cmake/ros.cmake)


SET(CUSTOM_HEADER_DIR "${PROJECT_NAME}")

SET(CXX_DISABLE_WERROR True)

SET(PKG_CONFIG_ADDITIONAL_VARIABLES
  ${PKG_CONFIG_ADDITIONAL_VARIABLES}
  plugindirname
  plugindir
  )


# Search for dependencies.

ADD_REQUIRED_DEPENDENCY("pinocchio >= 2.0.0")

ADD_PROJECT_DEPENDENCY(dynamic-graph 3.0.0 REQUIRED)
ADD_REQUIRED_DEPENDENCY("sot-core >= 4.0.0")
ADD_REQUIRED_DEPENDENCY("sot-dynamic-pinocchio >= 3.1")
ADD_REQUIRED_DEPENDENCY("dynamic-graph-python >= 3.0.0")

ADD_REQUIRED_DEPENDENCY("dynamic_graph_bridge_msgs")
ADD_REQUIRED_DEPENDENCY("dynamic_graph_bridge >= 3.0")

ADD_OPTIONAL_DEPENDENCY("talos_data")
ADD_OPTIONAL_DEPENDENCY("pyrene-motions")

# Search for dependencies.
# Boost

# Handle rpath necessary to handle ROS multiplace packages
# libraries inclusion
SET(CMAKE_INSTALL_LIBDIR lib)
SET(CMAKE_SKIP_BUILD_RPATH  FALSE)
SET(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}")
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# Add subdirectories.
ADD_SUBDIRECTORY(src)

IF(TALOS_DATA_FOUND)
  FOREACH(py_filename kinetalos katana_holding )
    CONFIGURE_FILE(
      ${PROJECT_SOURCE_DIR}/tests/${py_filename}.py.cmake
      ${PROJECT_BINARY_DIR}/tests/${py_filename}.py
      )
    INSTALL(FILES
      ${PROJECT_BINARY_DIR}/tests/${py_filename}.py
      DESTINATION
      ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/tests
      )
  ENDFOREACH(py_filename)

  IF(PYRENE_MOTIONS_FOUND)
    # Configure files using motions from pyrene-motions package
    FOREACH(py_filename appli-test-simple-seq-play )
      CONFIGURE_FILE(
        ${PROJECT_SOURCE_DIR}/tests/${py_filename}.py.cmake
        ${PROJECT_BINARY_DIR}/tests/${py_filename}.py
        )
      INSTALL(FILES
        ${PROJECT_BINARY_DIR}/tests/${py_filename}.py
        DESTINATION
        ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/tests
        )
    ENDFOREACH(py_filename)

    # Install python files starting the application
    FOREACH(py_filename test-simple-seq-play )
      INSTALL(FILES
        ${PROJECT_SOURCE_DIR}/tests/${py_filename}.py
        DESTINATION
        ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/tests
        )
    ENDFOREACH(py_filename)
  ENDIF(PYRENE_MOTIONS_FOUND)

  INSTALL(FILES
    ${PROJECT_SOURCE_DIR}/tests/test.py
    ${PROJECT_SOURCE_DIR}/tests/appli.py
    DESTINATION
    ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/tests
    )

ENDIF(TALOS_DATA_FOUND)

SETUP_PROJECT_FINALIZE()
