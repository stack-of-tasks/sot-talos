/* Copyright 2018, CNRS
 * Author: O. Stasse
 *
BSD 2-Clause License

Copyright (c) 2017, Stack Of Tasks development team
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 */

/**
\mainpage
\section sec_intro Introduction

This library implements a class called Device for the TALOS PAL-Robotics humanoid robots to be controlled with the
Stack-Of-Tasks. A derived class is used for the first prototype of this line of humanoid robot, Pyrene.

This class is implementing a perception-action loop.
To achieve this goal it is providing a Hardware Abstract Layer to communicate with the SoT control infra-structure and
the robot or a simulator. It is fully compatible with the roscontrol_sot package to run on a TALOS humanoid robot or
with the Gazebo simulator.

The sot-talos package contains also the class sot-talos-controller. This class is used to start the multithreading
environment which is handling ROS request, starts the python interpreter to control the SoT and finally handle the
control states: initialization, nominal usage, stopping or cancellation.

It also provides python scripts to be used to interact with the robot.

\section SoTTalosDevice

  The SoTTalosDevice provides the following output signals:
  <ul>
  <li> accelerometer </li>
  <li> gyrometer </li>
  <li> currents </li>
  <li> joint_angles </li>
  <li> motor_angles </li>
  <li> p_gains </li>
  <li> d_gains</li>
\section sot-talos-controller

*/
