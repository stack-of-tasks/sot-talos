# -*- coding: utf-8 -*-
# Copyright 2011, Florent Lamiraux, Thomas Moulard, JRL, CNRS/AIST

print("Prologue Pyrene TALOS Robot")

from dynamic_graph.entity import PyEntityFactoryClass
from dynamic_graph.sot.pyrene.robot import Robot

# Create the device.
# This entity behaves exactly like robotsimu except:
# 1. it does not provide the increment command
# 2. it forwards the robot control to the sot-abstract
#    controller.
def makeRobot ():
    DeviceTalos = PyEntityFactoryClass('DeviceTalos')

    # Create the robot using the device.
    robot = Robot(name = 'robot', device = DeviceTalos('PYRENE'))

    robot.dynamic.com.recompute (0)
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
