# Copyright 2016, Thomas Moulard, Olivier Stasse, JRL, CNRS/AIST
#
# This file is part of sot-talos.
# sot-talos is free software: you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation, either version 3 of
# the License, or (at your option) any later version.
#
# sot-talos is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# sot-talos. If not, see <http://www.gnu.org/licenses/>.

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/eigen.cmake)
INCLUDE(cmake/lapack.cmake)
INCLUDE(cmake/cpack.cmake)
INCLUDE(cmake/ros.cmake)

SET(PROJECT_NAME sot-talos)
SET(PROJECT_DESCRIPTION "dynamic-graph package for Talos robot")
SET(PROJECT_URL "http://github.com/stack-of-tasks/sot-talos")

SET(CUSTOM_HEADER_DIR "${PROJECT_NAME}")

SET(CXX_DISABLE_WERROR True)

SET(PKG_CONFIG_ADDITIONAL_VARIABLES
  ${PKG_CONFIG_ADDITIONAL_VARIABLES}
  plugindirname
  plugindir
  )

SETUP_PROJECT()

# Search for dependencies.

ADD_REQUIRED_DEPENDENCY("pinocchio")

ADD_REQUIRED_DEPENDENCY("dynamic-graph >= 3.0.0")
ADD_REQUIRED_DEPENDENCY("sot-core >= 3.0.0")
ADD_REQUIRED_DEPENDENCY("sot-dynamic-pinocchio >= 3.1")
ADD_REQUIRED_DEPENDENCY("dynamic-graph-python >= 3.0.0")

ADD_REQUIRED_DEPENDENCY("dynamic_graph_bridge_msgs")
ADD_REQUIRED_DEPENDENCY("dynamic_graph_bridge >= 3.0")

ADD_OPTIONAL_DEPENDENCY("talos-data")

# Search for dependencies.
# Boost
SET(BOOST_COMPONENTS filesystem system thread)
SEARCH_FOR_BOOST()
SEARCH_FOR_EIGEN()

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
  FOREACH(py_filename kinetalos katana_holding)
    CONFIGURE_FILE(
      ${PROJECT_SOURCE_DIR}/tests/${py_filename}.py.cmake
      ${PROJECT_SOURCE_DIR}/tests/${py_filename}.py
      )
    ENDFOREACH(file)
ENDIF(TALOS_DATA_FOUND)


SETUP_PROJECT_FINALIZE()
SETUP_PROJECT_CPACK()
