# Copyright 2016-2020, Thomas Moulard, Olivier Stasse, Guilhem Saurel, JRL, CNRS/AIST, LAAS-CNRS
#

CMAKE_MINIMUM_REQUIRED(VERSION 3.1)

# Project properties
SET(PROJECT_ORG stack-of-tasks)
SET(PROJECT_NAME sot-talos)
SET(PROJECT_DESCRIPTION "dynamic-graph package for Talos robot")
SET(PROJECT_URL "https://github.com/${PROJECT_ORG}/${PROJECT_NAME}")

# Project options
OPTION(SUFFIX_SO_VERSION "Suffix library name with its version" ON)

# Project configuration
SET(PROJECT_USE_CMAKE_EXPORT TRUE)
SET(CUSTOM_HEADER_DIR "sot/talos")
SET(CXX_DISABLE_WERROR TRUE)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/python.cmake)
INCLUDE(cmake/ros.cmake)

# Project definition
COMPUTE_PROJECT_ARGS(PROJECT_ARGS LANGUAGES CXX C)
PROJECT(${PROJECT_NAME} ${PROJECT_ARGS})

# Project dependencies
SET(BOOST_COMPONENTS filesystem system thread program_options
  unit_test_framework python)
SET(CATKIN_REQUIRED_COMPONENTS
  roscpp std_msgs dynamic_graph_bridge_msgs dynamic_graph_bridge)
SET(CATKIN_DEPENDS_LIBRARIES ros_bridge sot_loader)

ADD_PROJECT_DEPENDENCY(sot-dynamic-pinocchio REQUIRED)
ADD_PROJECT_DEPENDENCY(dynamic-graph-python REQUIRED)
ADD_PROJECT_DEPENDENCY(dynamic_graph_bridge REQUIRED)
ADD_PROJECT_DEPENDENCY(roscpp REQUIRED)
ADD_PROJECT_DEPENDENCY(talos_data REQUIRED)
ADD_PROJECT_DEPENDENCY(pyrene-motions REQUIRED)
ADD_REQUIRED_DEPENDENCY(dynamic_graph_bridge_msgs)

FINDPYTHON()
SEARCH_FOR_BOOST()
FIND_PACKAGE(catkin REQUIRED COMPONENTS ${CATKIN_REQUIRED_COMPONENTS})

ADD_SUBDIRECTORY(src)

FOREACH(py_filename kinetalos katana_holding)
  CONFIGURE_FILE(
    ${PROJECT_SOURCE_DIR}/tests/${py_filename}.py.cmake
    ${PROJECT_BINARY_DIR}/tests/${py_filename}.py)
  INSTALL(FILES
    ${PROJECT_BINARY_DIR}/tests/${py_filename}.py
    DESTINATION share/${PROJECT_NAME}/tests)
ENDFOREACH(py_filename)

# Configure files using motions from pyrene-motions package
FOREACH(py_filename appli-test-simple-seq-play)
CONFIGURE_FILE(
  ${PROJECT_SOURCE_DIR}/tests/${py_filename}.py.cmake
  ${PROJECT_BINARY_DIR}/tests/${py_filename}.py)
INSTALL(FILES
  ${PROJECT_BINARY_DIR}/tests/${py_filename}.py
  DESTINATION share/${PROJECT_NAME}/tests)
ENDFOREACH(py_filename)

# Install python files starting the application
FOREACH(py_filename test-simple-seq-play)
INSTALL(FILES
  ${PROJECT_SOURCE_DIR}/tests/${py_filename}.py
  DESTINATION share/${PROJECT_NAME}/tests)
ENDFOREACH(py_filename)

INSTALL(FILES
  ${PROJECT_SOURCE_DIR}/tests/test.py
  ${PROJECT_SOURCE_DIR}/tests/appli.py
  DESTINATION share/${PROJECT_NAME}/tests)

IF(NOT INSTALL_PYTHON_INTERFACE_ONLY)
  INSTALL(FILES package.xml DESTINATION share/${PROJECT_NAME})
ENDIF(NOT INSTALL_PYTHON_INTERFACE_ONLY)
