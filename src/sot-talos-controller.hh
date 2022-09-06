/*
 * Copyright 2016,
 *
 * Olivier Stasse
 *
 * LAAS, CNRS
 *
 * This file is part of TalosController.
 */

#ifndef _SOT_TalosController_H_
#define _SOT_TalosController_H_

#include <dynamic-graph/entity.h>
#include <dynamic-graph/linear-algebra.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/signal.h>

#include <dynamic_graph_bridge/ros_interpreter.hh>
#include <pinocchio/fwd.hpp>
#include <sot/core/abstract-sot-external-interface.hh>
#include <sot/core/device.hh>

#include "sot-talos-device.hh"
namespace dgsot = dynamicgraph::sot;

class SoTTalosController : public dgsot::AbstractSotExternalInterface {
 public:
  static const std::string LOG_PYTHON;

  SoTTalosController();
  SoTTalosController(const char robotName[]);
  SoTTalosController(std::string robotName);
  virtual ~SoTTalosController();

  void setupSetSensors(std::map<std::string, dgsot::SensorValues> &sensorsIn);

  void nominalSetSensors(std::map<std::string, dgsot::SensorValues> &sensorsIn);

  void cleanupSetSensors(std::map<std::string, dgsot::SensorValues> &sensorsIn);

  void getControl(std::map<std::string, dgsot::ControlValues> &anglesOut);

  void setNoIntegration(void);
  void setSecondOrderIntegration(void);

  /// Embedded python interpreter accessible via Corba/ros
  boost::shared_ptr<dynamicgraph::Interpreter> interpreter_;

  boost::shared_ptr<ros::NodeHandle> nh_;
  boost::shared_ptr<ros::AsyncSpinner> spinner_;

 protected:
  // Update output port with the control computed from the
  // dynamic graph.
  void updateRobotState(std::vector<double> &anglesIn);

  /// Run a python command
  void runPython(std::ostream &file, const std::string &command,
                 dynamicgraph::Interpreter &interpreter);

  virtual void startupPython();

  void init();

  SoTTalosDevice *device_;
};

#endif /* _SOT_TalosController_H_ */
