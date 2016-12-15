# Copyright 2016, R. Budhiraja, Olivier Stasse, CNRS
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

INCLUDE(../cmake/python.cmake)
FINDPYTHON()
INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_PATH})

FUNCTION(COMPILE_PLUGIN NAME SOURCES ENTITIES)
  ADD_LIBRARY(${NAME} SHARED ${SOURCES})
  SET_TARGET_PROPERTIES(${lib} PROPERTIES
    PREFIX ""
    SOVERSION ${PROJECT_VERSION})

  PKG_CONFIG_USE_DEPENDENCY(${NAME} dynamic-graph)
  PKG_CONFIG_USE_DEPENDENCY(${NAME} sot-core)

  INSTALL(TARGETS ${NAME} DESTINATION ${CMAKE_INSTALL_LIBDIR}/plugin)

  # build python submodule
  STRING(REPLACE - _ PYTHON_LIBRARY_NAME ${NAME})
  SET(NEW_ENTITY_CLASS ${ENTITIES})
  DYNAMIC_GRAPH_PYTHON_MODULE("sot/talos/${PYTHON_LIBRARY_NAME}"
    ${NAME}
    sot-talos-${PYTHON_LIBRARY_NAME}-wrap
    )
ENDFUNCTION()

# Compile plug-ins.
# COMPILE_PLUGIN(dynamic-talos dynamic-talos.cpp DynamicTalos)


## 
##CONFIG_FILES(dynamic_graph/sot/talos/robot.py)

# Install Python files.
SET(PYTHON_MODULE_DIR
  ${CMAKE_CURRENT_SOURCE_DIR}/dynamic_graph/sot/talos)
SET(PYTHON_MODULE_BUILD_DIR
  ${CMAKE_CURRENT_BINARY_DIR}/dynamic_graph/sot/talos)

SET(PYTHON_MODULE dynamic_graph/sot/talos)
PYTHON_INSTALL_ON_SITE("${PYTHON_MODULE}" "__init__.py" )
PYTHON_INSTALL_ON_SITE("${PYTHON_MODULE}" "talos.py" )
SET(FILES __init__.py robot.py)

# Add the library to wrap the device of Talos.
MACRO(build_talos_device)

  SET(DEVICE_NAME sot-talos-device)
  ADD_LIBRARY(${DEVICE_NAME}
    SHARED
    sot-talos-device.cpp
  )

  # Link the dynamic library containing the SoT with its dependencies.
  PKG_CONFIG_USE_DEPENDENCY(${DEVICE_NAME} "dynamic-graph")
  PKG_CONFIG_USE_DEPENDENCY(${DEVICE_NAME} "dynamic-graph-python")
  PKG_CONFIG_USE_DEPENDENCY(${DEVICE_NAME} "sot-core")

  PKG_CONFIG_USE_DEPENDENCY(${DEVICE_NAME} "dynamic_graph_bridge")

  IF(UNIX AND NOT APPLE)
    TARGET_LINK_LIBRARIES(${DEVICE_NAME} ${Boost_LIBRARIES})
  ENDIF(UNIX AND NOT APPLE)

  INSTALL(TARGETS ${DEVICE_NAME} DESTINATION ${CMAKE_INSTALL_LIBDIR})

  # build python submodule
  STRING(REPLACE - _ PYTHON_LIBRARY_NAME ${DEVICE_NAME})
  SET(NEW_ENTITY_CLASS ${ENTITIES})
  DYNAMIC_GRAPH_PYTHON_MODULE("sot/talos/${PYTHON_LIBRARY_NAME}"
    ${DEVICE_NAME}
    sot-talos-${PYTHON_LIBRARY_NAME}-wrap
    )

ENDMACRO()

build_talos_device()

# Add the library to wrap the controller of Pyrene.
MACRO(build_talos_controller robotname)

  SET(CONTROLLER_NAME sot-${robotname}-controller)
  ADD_LIBRARY(${CONTROLLER_NAME}
    SHARED
    sot-talos-controller.cpp
    sot-${robotname}-controller.cpp
  )

  # Link the dynamic library containing the SoT with its dependencies.
  PKG_CONFIG_USE_DEPENDENCY(${CONTROLLER_NAME} "dynamic-graph")
  PKG_CONFIG_USE_DEPENDENCY(${CONTROLLER_NAME} "dynamic-graph-python")
  PKG_CONFIG_USE_DEPENDENCY(${CONTROLLER_NAME} "sot-core")

  PKG_CONFIG_USE_DEPENDENCY(${CONTROLLER_NAME} "dynamic_graph_bridge")

  ADD_DEPENDENCIES(${CONTROLLER_NAME} "sot-hrp2-device")
  TARGET_LINK_LIBRARIES(${CONTROLLER_NAME} "sot-hrp2-device")
  TARGET_LINK_LIBRARIES(${CONTROLLER_NAME} "ros_bridge")
  TARGET_LINK_LIBRARIES(${CONTROLLER_NAME} "ros_interpreter")
  
  IF(UNIX AND NOT APPLE)
    TARGET_LINK_LIBRARIES(${CONTROLLER_NAME} ${Boost_LIBRARIES})
  ENDIF(UNIX AND NOT APPLE)

  INSTALL(TARGETS ${CONTROLLER_NAME} DESTINATION ${CMAKE_INSTALL_LIBDIR})

ENDMACRO()

build_talos_controller("pyrene")

