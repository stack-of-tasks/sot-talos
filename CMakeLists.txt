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
SET(PROJECT_URL "http://github.com/laas/sot-talos")

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
ADD_REQUIRED_DEPENDENCY("sot-dynamic >= 3.1")
ADD_REQUIRED_DEPENDENCY("dynamic-graph-python >= 3.0.0")

ADD_REQUIRED_DEPENDENCY("dynamic_graph_bridge_msgs")
ADD_REQUIRED_DEPENDENCY("dynamic_graph_bridge >= 3.0")

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
#ADD_SUBDIRECTORY(tests)

SETUP_PROJECT_FINALIZE()
SETUP_PROJECT_CPACK()
cmake_minimum_required(VERSION 2.6)

include(cmake/base.cmake)
include(cmake/pthread.cmake)
include(cmake/eigen.cmake)

set(PROJECT_NAME sot-talos)
set(PROJECT_DESCRIPTION "Sot Talos")
set(PROJECT_URL "https://github.com/stack-of-tasks/sot-talos")

# Disable -Werror on Unix for now.
set(CXX_DISABLE_WERROR True)
set(CMAKE_VERBOSE_MAKEFILE True)

# Add std=c++11 flag
if (UNIX)
  SET(LFLAG -std=c++11)
  SET(CMAKE_CXX_FLAGS "${LFLAG} ${CMAKE_CXX_FLAGS}")
endif(UNIX)

setup_project()

if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
  #cmake_policy(SET CMP0053 OLD)
endif(COMMAND cmake_policy)

add_required_dependency("dynamic-graph")
add_required_dependency("dynamic-graph-python")
add_required_dependency("sot-core")
add_required_dependency("dynamic_graph_bridge")

search_for_eigen()

# Subdirectories
add_subdirectory(src)

setup_project_finalize()
