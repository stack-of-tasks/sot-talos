# -*- coding: utf-8 -*-
# Copyright 2016, Olivier STASSE, LAAS-CNRS

from __future__ import print_function

import pinocchio

from dynamic_graph import plug
from dynamic_graph.sot.core.math_small_entities import Derivator_of_Vector
from dynamic_graph.sot.dynamic_pinocchio import DynamicPinocchio
from dynamic_graph.sot.dynamic_pinocchio.humanoid_robot import \
    AbstractHumanoidRobot
from dynamic_graph.sot.core.parameter_server import ParameterServer


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

    forceSensorInLeftAnkle = ((1., 0., 0., 0.), (0., 1., 0., 0.), (0., 0., 1., -0.107), (0., 0., 0., 1.))
    forceSensorInRightAnkle = ((1., 0., 0., 0.), (0., 1., 0., 0.), (0., 0., 1., -0.107), (0., 0., 0., 1.))
    defaultFilename = "package://talos_data/urdf/talos_reduced_v2.urdf"
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
        # Gripper position in full configuration: 27:34, and 41:48
        # Small configuration: 36 DOF
        # Full configuration: 50 DOF
        res = config[0:27] + 7 * (0., ) + config[27:34] + 7 * (0., ) + config[34:]
        return res

    def __init__(self, name, device=None, tracer=None, fromRosParam=False):
        self.OperationalPointsMap = {
            'left-wrist': 'arm_left_7_joint',
            'right-wrist': 'arm_right_7_joint',
            'left-ankle': 'leg_left_6_joint',
            'right-ankle': 'leg_right_6_joint',
            'gaze': 'head_2_joint',
            'waist': 'root_joint',
            'chest': 'torso_2_joint'
        }

        if fromRosParam:
            ltimeStep=0.005
            if device is not None:
                ltimeStep = device.getTimeStep()

            print("Using SoT parameter \"/robot_description\"")
            paramName = "/robot_description"
            self.param_server = ParameterServer("param_server")
            self.param_server.init_simple(ltimeStep)
            model2_string=self.param_server.getParameter(paramName)
            self.loadModelFromString(model2_string,
                                     rootJointType=pinocchio.JointModelFreeFlyer,
                                     removeMimicJoints=True)
        else:
            self.loadModelFromUrdf(self.defaultFilename,
                                   rootJointType=pinocchio.JointModelFreeFlyer,
                                   removeMimicJoints=True)

        assert hasattr(self, "pinocchioModel")
        assert hasattr(self, "pinocchioData")

        if device is not None:
            self.device = device
        AbstractHumanoidRobot.__init__(self, name, tracer)

        self.OperationalPoints.append('waist')
        self.OperationalPoints.append('chest')

        # Create rigid body dynamics model and data (pinocchio)
        self.dynamic = DynamicPinocchio(self.name + "_dynamic")
        self.dynamic.setModel(self.pinocchioModel)
        self.dynamic.setData(self.pinocchioData)
        self.dynamic.displayModel()
        self.dimension = self.dynamic.getDimension()

        self.initializeRobot()

        self.AdditionalFrames.append(
            ("leftFootForceSensor", self.forceSensorInLeftAnkle, self.OperationalPointsMap["left-ankle"]))
        self.AdditionalFrames.append(
            ("rightFootForceSensor", self.forceSensorInRightAnkle, self.OperationalPointsMap["right-ankle"]))

        # Create operational points based on operational points map (if provided)
        if self.OperationalPointsMap is not None:
            self.initializeOpPoints()


__all__ = [Talos]
