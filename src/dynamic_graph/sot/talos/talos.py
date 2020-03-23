# -*- coding: utf-8 -*-
# Copyright 2016, Olivier STASSE, LAAS-CNRS

from __future__ import print_function

from dynamic_graph import plug
from dynamic_graph.sot.core.math_small_entities import Derivator_of_Vector
from dynamic_graph.sot.dynamic_pinocchio import DynamicPinocchio
from dynamic_graph.sot.dynamic_pinocchio.humanoid_robot import AbstractHumanoidRobot
import pinocchio
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

    def __init__(self, name, initialConfig, device=None, tracer=None, fromRosParam=False):
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
            print("Using ROS parameter \"/robot_description\"")
            rosParamName = "/robot_description"
            import rospy
            if rosParamName not in rospy.get_param_names():
                raise RuntimeError('"' + rosParamName + '" is not a ROS parameter.')
            s = rospy.get_param(rosParamName)

            self.loadModelFromString(s, rootJointType=pinocchio.JointModelFreeFlyer,
                    removeMimicJoints=True)
        else:
            self.loadModelFromUrdf(self.defaultFilename,
                    rootJointType=pinocchio.JointModelFreeFlyer,
                    removeMimicJoints=True)

        assert hasattr(self, "pinocchioModel")
        assert hasattr(self, "pinocchioData")

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
