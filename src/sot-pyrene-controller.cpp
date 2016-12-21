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
 */

#include <sot/core/debug.hh>

/* Pyrene is the first TALOS */
#define ROBOTNAME std::string("PYRENE")

#include "sot-pyrene-controller.hh"

const std::string SoTPyreneController::LOG_PYTHON_PYRENE="/tmp/PyreneController_python.out";

SoTPyreneController::SoTPyreneController():
  SoTTalosController(ROBOTNAME)
{
  startupPython();
  interpreter_->startRosService ();
}

void SoTPyreneController::startupPython()
{
  SoTTalosController::startupPython();
  std::ofstream aof(LOG_PYTHON_PYRENE.c_str());
  runPython
    (aof,
     "from dynamic_graph.sot.pyrene.prologue import robot",
     *interpreter_);
  aof.close();
}

extern "C" 
{
  dgsot::AbstractSotExternalInterface * createSotExternalInterface()
  {
    return new SoTPyreneController;
  }
}

extern "C"
{
  void destroySotExternalInterface(dgsot::AbstractSotExternalInterface *p)
  {
    delete p;
  }
}
