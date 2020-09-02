# -*- coding: utf-8 -*-
# Copyright 2011, Florent Lamiraux, Thomas Moulard, JRL, CNRS/AIST

# sys.argv is not defined when running the remove interpreter, but it is
# required by rospy
import sys

import numpy as np

from dynamic_graph.sot.pyrene.robot import Robot
from dynamic_graph.sot.talos.sot_talos_device import DeviceTalos

if not hasattr(sys, 'argv'):
    sys.argv = [
        "dynamic_graph",
    ]

print("Prologue Pyrene TALOS Robot")


def makeRobot():
    """
    Create the device.
    This entity behaves exactly like robotsimu except:
    1. it does not provide the increment command
    2. it forwards the robot control to the sot-abstract
       controller.
    """

    # Create the robot using the device.
    robot = Robot(name='talos', device=DeviceTalos('PYRENE'), fromRosParam=True)
    robot.dynamic.com.recompute(0)
    _com = robot.dynamic.com.value
    robot.device.zmp.value = np.array((_com[0], _com[1], 0.))

    return robot


####################################
#        --- IMPORTANT ---         #
#                                  #
# THIS FILE MUST NEVER BE CHANGED. #
# TO RUN YOUR EXPERIMENT, PLEASE   #
# WRITE A SEPARATE PYTHON MODULE   #
# AND LAUNCH IT USING dg-remote!   #
####################################
