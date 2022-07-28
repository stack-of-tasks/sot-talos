# Copyright 2016-2020, Thomas Moulard, Olivier Stasse, Guilhem Saurel, JRL,
# CNRS/AIST, LAAS-CNRS
#

cmake_minimum_required(VERSION 3.1)

# Project properties
set(PROJECT_ORG stack-of-tasks)
set(PROJECT_NAME sot-talos)
set(PROJECT_DESCRIPTION "dynamic-graph package for Talos robot")
set(PROJECT_URL "https://github.com/${PROJECT_ORG}/${PROJECT_NAME}")

# Project options
option(SUFFIX_SO_VERSION "Suffix library name with its version" ON)

# Project configuration
set(PROJECT_USE_CMAKE_EXPORT TRUE)
set(CUSTOM_HEADER_DIR "sot/talos")
set(CXX_DISABLE_WERROR TRUE)

# JRL-cmakemodule setup
include(cmake/base.cmake)
include(cmake/boost.cmake)
include(cmake/python.cmake)
include(cmake/ros.cmake)

# Project definition
compute_project_args(PROJECT_ARGS LANGUAGES CXX C)
project(${PROJECT_NAME} ${PROJECT_ARGS})

# Project dependencies
findpython()
search_for_boost_python()

set(CATKIN_REQUIRED_COMPONENTS roscpp std_msgs)
set(CATKIN_DEPENDS_LIBRARIES ros_bridge sot_loader)

add_project_dependency(sot-dynamic-pinocchio REQUIRED)
add_project_dependency(dynamic-graph-python 4.0.0 REQUIRED)
add_project_dependency(dynamic_graph_bridge REQUIRED)
add_project_dependency(roscpp REQUIRED)
add_project_dependency(talos_data REQUIRED)
add_project_dependency(pyrene-motions REQUIRED)
add_required_dependency(dynamic_graph_bridge_msgs)

find_package(catkin REQUIRED COMPONENTS ${CATKIN_REQUIRED_COMPONENTS})

add_subdirectory(src)

foreach(py_filename kinetalos katana_holding)
  configure_file(${PROJECT_SOURCE_DIR}/tests/${py_filename}.py.cmake
                 ${PROJECT_BINARY_DIR}/tests/${py_filename}.py)
  install(FILES ${PROJECT_BINARY_DIR}/tests/${py_filename}.py
          DESTINATION share/${PROJECT_NAME}/tests)
endforeach(py_filename)

# Configure files using motions from pyrene-motions package
foreach(py_filename appli-test-simple-seq-play)
  configure_file(${PROJECT_SOURCE_DIR}/tests/${py_filename}.py.cmake
                 ${PROJECT_BINARY_DIR}/tests/${py_filename}.py)
  install(FILES ${PROJECT_BINARY_DIR}/tests/${py_filename}.py
          DESTINATION share/${PROJECT_NAME}/tests)
endforeach(py_filename)

# Install python files starting the application
foreach(py_filename test-simple-seq-play)
  install(FILES ${PROJECT_SOURCE_DIR}/tests/${py_filename}.py
          DESTINATION share/${PROJECT_NAME}/tests)
endforeach(py_filename)

install(FILES ${PROJECT_SOURCE_DIR}/tests/test.py
              ${PROJECT_SOURCE_DIR}/tests/appli.py
        DESTINATION share/${PROJECT_NAME}/tests)

if(NOT INSTALL_PYTHON_INTERFACE_ONLY)
  install(FILES package.xml DESTINATION share/${PROJECT_NAME})
endif(NOT INSTALL_PYTHON_INTERFACE_ONLY)
