# -*- coding: utf-8 -*-
# Copyright 2011, Florent Lamiraux, Thomas Moulard, JRL, CNRS/AIST

# sys.argv is not defined when running the remove interpreter, but it is
# required by rospy
import sys

from dynamic_graph.entity import PyEntityFactoryClass
from dynamic_graph.sot.pyrene.robot import Robot

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

    DeviceTalos = PyEntityFactoryClass('DeviceTalos')

    # Create the robot using the device.
    robot = Robot(name='talos', device=DeviceTalos('PYRENE'),
                  fromRosParam=True)
    robot.dynamic.com.recompute(0)
    _com = robot.dynamic.com.value
    robot.device.zmp.value = (_com[0], _com[1], 0.)

    return robot


####################################
#        --- IMPORTANT ---         #
#                                  #
# THIS FILE MUST NEVER BE CHANGED. #
# TO RUN YOUR EXPERIMENT, PLEASE   #
# WRITE A SEPARATE PYTHON MODULE   #
# AND LAUNCH IT USING dg-remote!   #
####################################
