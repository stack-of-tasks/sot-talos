# Copyright 2016, 2020, R. Budhiraja, Olivier Stasse, Guilhem SaurelCNRS

# Install Python files.

set(PYTHON_MODULE dynamic_graph/sot/talos)
python_install_on_site("${PYTHON_MODULE}" "__init__.py")
python_install_on_site("${PYTHON_MODULE}" "talos.py")
set(FILES __init__.py robot.py)

# Install dynamic_graph.sot.pyrene
set(PYTHON_MODULE dynamic_graph/sot/pyrene)
python_install_on_site("${PYTHON_MODULE}" "__init__.py")
python_install_on_site("${PYTHON_MODULE}" "prologue.py")
python_install_on_site("${PYTHON_MODULE}" "robot.py")

# Add the library to wrap the device of Talos.
set(DEVICE_NAME sot-talos-device)
add_library(${DEVICE_NAME} SHARED sot-talos-device.cpp)

# Link the dynamic library containing the SoT with its dependencies.
target_link_libraries(
  ${DEVICE_NAME} sot-core::sot-core dynamic-graph-python::dynamic-graph-python
  dynamic_graph_bridge::ros_bridge)

install(
  TARGETS ${DEVICE_NAME}
  EXPORT ${TARGETS_EXPORT_NAME}
  DESTINATION lib)

# build python submodule
string(REPLACE - _ PYTHON_LIBRARY_NAME ${DEVICE_NAME})
dynamic_graph_python_module(
  "sot/talos/${PYTHON_LIBRARY_NAME}" ${DEVICE_NAME}
  sot-talos-${PYTHON_LIBRARY_NAME}-wrap SOURCE_PYTHON_MODULE
  "${CMAKE_CURRENT_SOURCE_DIR}/${DEVICE_NAME}-python-module-py.cc")

# Add the library to wrap the controller of Pyrene.
set(CONTROLLER_NAME sot-pyrene-controller)
add_library(${CONTROLLER_NAME} SHARED ${CONTROLLER_NAME}.cpp
                                      sot-talos-controller.cpp)

# Link the dynamic library containing the SoT with its dependencies.
target_link_libraries(
  ${CONTROLLER_NAME} ${DEVICE_NAME} dynamic_graph_bridge::ros_interpreter
  dynamic_graph_bridge::ros_bridge)
pkg_config_use_dependency(${CONTROLLER_NAME} dynamic_graph_bridge_msgs)

install(
  TARGETS ${CONTROLLER_NAME}
  EXPORT ${TARGETS_EXPORT_NAME}
  DESTINATION lib)
