# Copyright 2016, 2020, R. Budhiraja, Olivier Stasse, Guilhem SaurelCNRS

# Install Python files.

SET(PYTHON_MODULE dynamic_graph/sot/talos)
PYTHON_INSTALL_ON_SITE("${PYTHON_MODULE}" "__init__.py")
PYTHON_INSTALL_ON_SITE("${PYTHON_MODULE}" "talos.py")
SET(FILES __init__.py robot.py)

# Install dynamic_graph.sot.pyrene
SET(PYTHON_MODULE dynamic_graph/sot/pyrene)
PYTHON_INSTALL_ON_SITE("${PYTHON_MODULE}" "__init__.py")
PYTHON_INSTALL_ON_SITE("${PYTHON_MODULE}" "prologue.py")
PYTHON_INSTALL_ON_SITE("${PYTHON_MODULE}" "robot.py")

# Add the library to wrap the device of Talos.
SET(DEVICE_NAME sot-talos-device)
ADD_LIBRARY(${DEVICE_NAME} SHARED sot-talos-device.cpp)

# Link the dynamic library containing the SoT with its dependencies.
TARGET_LINK_LIBRARIES(${DEVICE_NAME}
  sot-core::sot-core
  dynamic-graph-python::dynamic-graph-python
  dynamic_graph_bridge::ros_bridge)

INSTALL(TARGETS ${DEVICE_NAME} EXPORT ${TARGETS_EXPORT_NAME} DESTINATION lib)

# build python submodule
STRING(REPLACE - _ PYTHON_LIBRARY_NAME ${DEVICE_NAME})
SET(NEW_ENTITY_CLASS ${ENTITIES})
DYNAMIC_GRAPH_PYTHON_MODULE("sot/talos/${PYTHON_LIBRARY_NAME}"
  ${DEVICE_NAME} sot-talos-${PYTHON_LIBRARY_NAME}-wrap)

# Add the library to wrap the controller of Pyrene.
SET(CONTROLLER_NAME sot-pyrene-controller)
ADD_LIBRARY(${CONTROLLER_NAME} SHARED ${CONTROLLER_NAME}.cpp
  sot-talos-controller.cpp)

# Link the dynamic library containing the SoT with its dependencies.
TARGET_LINK_LIBRARIES(${CONTROLLER_NAME} ${DEVICE_NAME}
  dynamic_graph_bridge::ros_interpreter
  dynamic_graph_bridge::ros_bridge)
PKG_CONFIG_USE_DEPENDENCY(${CONTROLLER_NAME} dynamic_graph_bridge_msgs)

INSTALL(TARGETS ${CONTROLLER_NAME} EXPORT ${TARGETS_EXPORT_NAME} DESTINATION lib)
