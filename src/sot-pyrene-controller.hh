/*
 * Copyright 2016,
 *
 * Olivier Stasse
 *
 * LAAS, CNRS
 *
 * This file is part of TALOSController.
 * TALOSController is a free software,
 *
 *
 */

#ifndef _SOT_PYRENE_Controller_H_
#define _SOT_PYRENE_Controller_H_

#include "sot-talos-controller.hh"
namespace dgsot = dynamicgraph::sot;

class SoTPyreneController : public SoTTalosController {
 public:
  static const std::string LOG_PYTHON_PYRENE;

  SoTPyreneController();
  virtual ~SoTPyreneController(){};

 protected:
  virtual void startupPython();
};

#endif /* _SOTTalosController_H_ */
