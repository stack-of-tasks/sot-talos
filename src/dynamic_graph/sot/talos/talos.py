# -*- coding: utf-8 -*-
# Copyright 2016, Olivier STASSE, LAAS-CNRS
#
# This file is part of TALOSController.
# TALOSController is free software: you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation, either version 3 of
# the License, or (at your option) any later version.
#
# TALOSController is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# TALOSController. If not, see <http://www.gnu.org/licenses/>.

from __future__ import print_function

import numpy as np

from dynamic_graph.sot.dynamics_pinocchio.humanoid_robot import AbstractHumanoidRobot
from dynamic_graph.ros import RosRobotModel
import pinocchio as se3
from rospkg import RosPack

# Internal helper tool.
def matrixToTuple(M):
    tmp = M.tolist()
    res = []
    for i in tmp:
        res.append(tuple(i))
    return tuple(res)

class Talos(AbstractHumanoidRobot):
    """
    This class defines a Talos robot
    """

    forceSensorInLeftAnkle =  ((1.,0.,0.,0.),
                               (0.,1.,0.,0.),
                               (0.,0.,1.,-0.107),
                               (0.,0.,0.,1.))
    forceSensorInRightAnkle = ((1.,0.,0.,0.),
                               (0.,1.,0.,0.),
                               (0.,0.,1.,-0.107),
                               (0.,0.,0.,1.))
    """
    TODO: Confirm the position and existence of these sensors
    accelerometerPosition = np.matrix ((
            (1., 0., 0., -.13,),
            (0., 1., 0., 0.,),
            (0., 0., 1., .118,),
            (0., 0., 0., 1.,),
            ))

    gyrometerPosition = np.matrix ((
            (1., 0., 0., -.13,),
            (0., 1., 0., 0.,),
            (0., 0., 1., .118,),
            (0., 0., 0., 1.,),
            ))
    """
    def smallToFull(self, config):
        #Gripper position in full configuration: 27:34, and 41:48
        #Small configuration: 36 DOF
        #Full configuration: 50 DOF
        res = config[0:27] + 7*(0.,) + config[27:34]+ 7*(0.,)+config[34:]
        return res

    def __init__(self, name, device = None, tracer = None):
        AbstractHumanoidRobot.__init__ (self, name, tracer)
        self.OperationalPoints.append('waist')
        self.OperationalPoints.append('chest')
        self.device = device

        """
        TODO: Confirm these sensors
        self.AdditionalFrames.append(
            ("accelerometer",
             matrixToTuple(self.accelerometerPosition), "chest"))
        self.AdditionalFrames.append(
            ("gyrometer",
             matrixToTuple(self.gyrometerPosition), "chest"))
        """

        self.OperationalPointsMap = {'left-wrist'  : 'arm_left_7_joint',
                                     'right-wrist' : 'arm_right_7_joint',
                                     'left-ankle'  : 'leg_left_5_joint',
                                     'right-ankle' : 'leg_right_5_joint',
                                     'gaze'        : 'head_2_joint',
                                     'waist'       : 'root_joint',
                                     'chest'       : 'torso_2_joint'}

        self.AdditionalFrames.append(
            ("leftFootForceSensor",
             self.forceSensorInLeftAnkle, self.OperationalPointsMap("left-ankle")))
        self.AdditionalFrames.append(
            ("rightFootForceSensor",
             self.forceSensorInRightAnkle, self.OperationalPointsMap("right-ankle")))

        self.dynamic = RosRobotModel("{0}_dynamic".format(name))
        rospack = RosPack()
        self.urdfPath = rospack.get_path('talos_description') + '/urdf/talos.urdf'

        self.pinocchioModel = se3.buildModelFromUrdf(self.urdfPath, se3.JointModelFreeFlyer())
        self.pinocchioData = self.pinocchioModel.createData()
        self.dynamic.setModel(self.pinocchioModel)
        self.dynamic.setData(self.pinocchioData)
        self.dimension = self.dynamic.getDimension()
        self.plugVelocityFromDevice = True
        if self.dimension != len(self.halfSitting):
            raise RuntimeError("Dimension of half-sitting: {0} differs from dimension of robot: {1}".format (len(self.halfSitting), self.dimension))
        self.initializeRobot()
        self.dynamic.displayModel()
__all__ = [Talos]
