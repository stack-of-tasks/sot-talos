# -*- coding: utf-8 -*-
# Copyright 2011, Florent Lamiraux, Thomas Moulard, JRL, CNRS/AIST
#
# This file is part of dynamic-graph.
# dynamic-graph is free software: you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation, either version 3 of
# the License, or (at your option) any later version.
#
# dynamic-graph is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# dynamic-graph. If not, see <http://www.gnu.org/licenses/>.

from dynamic_graph.sot.dynamics.humanoid_robot import AbstractHumanoidRobot
from dynamic_graph.ros import RosRobotModel
import pinocchio as se3
from rospkg import RosPack

# Sot model for the talos_small.urdf (with gripper, no fingers)
class Robot (AbstractHumanoidRobot):
    """
    This class instantiates Talos Pyrene
    """
    # half sitting position for the given  urdf model
    # coordinates should be given in depth 
    # moreover, we do not consider fixed joint
    """
    halfSittingSmall = (
        # Free flyer
        0, 0, 0.840252, 0, 0, 0,
        # ImuTorsoAccelerometer_joint
        #0.0,
        # ImuTorsoGyrometer_joint
        #0.0,
        # LHipYaw, LHipRoll, LHipPtch, LKneePitch, LAnklePitch, L_AnkleRoll
        0.0, 0.0, -0.3490658, 0.6981317, -0.3490658, 0.0,
        # LFsrCenterJoint, LFsrFL_joint, LFsrFR_joint, LFsrRCenter_joint,
        #0.0, 0.0, 0.0, 0.0,
        # l_sole_joint
        #0.0, 
        # RHipYaw, RHipRoll, RHipPtch, RKneePitch, RAnklePitch, R_AnkleRoll
        0.0, 0.0, -0.3490658, 0.6981317, -0.3490658, 0.0,
        # RFsrCenterJoint, RFsrFL_joint, RFsrFR_joint, RFsrRCenter_joint,
        #0.0, 0.0, 0.0, 0.0,
        # r_sole_joint
        #0.0,
        # TrunkYaw
        0.0,
        # LShoulderPitch, LShoulderYaw, LElbowRoll, LElbowYaw, 
        1.5, 0.6, -0.5, -1.05,
        # LWristRoll, LWristYaw, LWristPitch,
        -0.4, -0.3, -0.2,
        # LFinger21, LFinger22, LFinger23
        #0.0, 0.0, 0.0,
        # LFinger31, LFinger32, LFinger33,
        #0.0, 0.0, 0.0, 
        # LHand,
        #0.0,
        # LFinger12, LFinger13
        #0.0, 0.0,
        # LThumb1, LThumb2, LThumb3
        #0.0, 0.0, 0.0,
        # l_gripper_joint
        #0.0,
        # NeckYaw, NeckPitch, 
        0.0, 0.0,
        # HeadPitch, HeadRoll
        0.0, 0.0, 
        # CameraDepth_joint, 
        #0.0,
        # CameraLeftEye_joint, CameraLeft_joint, 
        #0.0, 0.0,
        # CameraRightEye_joint, CameraRight_joint,
        #0.0, 0.0,
        # HeadTouchFront_joint, HeadTouchMiddle_joint
        #0.0, 0.0,
        # ImuHeadAccelerometer_joint, ImuHeadGyrometer_joint
        #0.0, 0.0,
        # gaze_joint
        #0.0,
        # RShoulderPitch, RShoulderYaw, RElbowRoll, RElbowYaw, 
        1.5, -0.6, 0.5, 1.05,
        # RWristRoll, RWristYaw, RWristPitch,
        -0.4, -0.3, -0.2,
        # RFinger21, RFinger22, RFinger23
        #0.0, 0.0, 0.0,
        # RFinger31, RFinger32, RFinger33,
        #0.0, 0.0, 0.0,
        # RHand,
        #0.0,  
        #  RFinger12, RFinger13
        # 0.0, 0.0, 
        # RThumb1, RThumb2, RThumb3
        #0.0, 0.0, 0.0,
        # r_gripper_joint
        #0.0,
    )

    # half sitting position for the talos urdf model
    # coordinates should be given in depth 
    # moreover, we do not consider fixed joint
    halfSittingAll = (
        # Free flyer
        0, 0, 0.840252, 0, 0, 0,
        # ImuTorsoAccelerometer_joint
        #0.0,
        # ImuTorsoGyrometer_joint
        #0.0,
        # LHipYaw, LHipRoll, LHipPtch, LKneePitch, LAnklePitch, L_AnkleRoll
        0.0, 0.0, -0.3490658, 0.6981317, -0.3490658, 0.0,
        # LFsrCenterJoint, LFsrFL_joint, LFsrFR_joint, LFsrRCenter_joint,
        #0.0, 0.0, 0.0, 0.0,
        # l_sole_joint
        #0.0, 
        # RHipYaw, RHipRoll, RHipPtch, RKneePitch, RAnklePitch, R_AnkleRoll
        0.0, 0.0, -0.3490658, 0.6981317, -0.3490658, 0.0,
        # RFsrCenterJoint, RFsrFL_joint, RFsrFR_joint, RFsrRCenter_joint,
        #0.0, 0.0, 0.0, 0.0,
        # r_sole_joint
        #0.0,
        # TrunkYaw
        0.0,
        # LShoulderPitch, LShoulderYaw, LElbowRoll, LElbowYaw, 
        1.5, 0.6, -0.5, -1.05,
        # LWristRoll, LWristYaw, LWristPitch,
        -0.4, -0.3, -0.2,
        # LFinger21, LFinger22, LFinger23
        0.0, 0.0, 0.0,
        # LFinger31, LFinger32, LFinger33,
        0.0, 0.0, 0.0, 
        # LHand, LFinger12, LFinger13
        0.0, 0.0, 0.0,
        # LThumb1, LThumb2, LThumb3
        0.0, 0.0, 0.0,
        # l_gripper_joint
        #0.0,
        # NeckYaw, NeckPitch, 
        0.0, 0.0,
        # HeadPitch, HeadRoll
        0.0, 0.0, 
        # CameraDepth_joint, 
        #0.0,
        # CameraLeftEye_joint, CameraLeft_joint, 
        #0.0, 0.0,
        # CameraRightEye_joint, CameraRight_joint,
        #0.0, 0.0,
        # HeadTouchFront_joint, HeadTouchMiddle_joint
        #0.0, 0.0,
        # ImuHeadAccelerometer_joint, ImuHeadGyrometer_joint
        #0.0, 0.0,
        # gaze_joint
        #0.0,
        # RShoulderPitch, RShoulderYaw, RElbowRoll, RElbowYaw, 
        1.5, -0.6, 0.5, 1.05,
        # RWristRoll, RWristYaw, RWristPitch,
        -0.4, -0.3, -0.2,
        # RFinger21, RFinger22, RFinger23
        0.0, 0.0, 0.0,
        # RFinger31, RFinger32, RFinger33,
        0.0, 0.0, 0.0,
        # RHead, RFinger12, RFinger13
        0.0, 0.0, 0.0, 
        # RThumb1, RThumb2, RThumb3
        0.0, 0.0, 0.0,
        # r_gripper_joint
        #0.0,
    )
    """

    def __init__(self, name, 
                 device = None,
                 tracer = None):
        AbstractHumanoidRobot.__init__ (self, name, tracer)

        print "You ask for ", name, ". "
        rospack = RosPack()
        self.urdfDir = rospack.get_path('talos_description') + '/urdf/'
        if name == 'talos_small':
            print "Loaded model is talos_small.urdf."
            self.urdfName = 'talos_small.urdf'
            self.halfSitting = self.halfSittingSmall
        else:
            print "Loaded model is talos.urdf."
            self.urdfName = 'talos.urdf'
            self.halfSitting = self.halfSittingAll

        self.OperationalPoints.append('waist')
        self.OperationalPoints.append('chest')
        """
        self.OperationalPointsMap = {'left-wrist'  : 'LWristPitch',
                                     'right-wrist' : 'RWristPitch',
                                     'left-ankle'  : 'LAnkleRoll',
                                     'right-ankle' : 'RAnkleRoll',
                                     'gaze'        : 'gaze_joint',
                                     'waist'       : 'waist',
                                     'chest'       : 'TrunkYaw'}
        """
        self.device = device

        # correct the name of the body link
        self.dynamic = RosRobotModel("{0}_dynamic".format(name))

        self.pinocchioModel = se3.buildModelFromUrdf(self.urdfDir + self.urdfName,
                                                     se3.JointModelFreeFlyer())
        self.pinocchioData = self.pinocchioModel.createData()
        self.dynamic.setModel(self.pinocchioModel)
        self.dynamic.setData(self.pinocchioData)

        # complete feet position (TODO: move it into srdf file)
        #ankle =self.dynamic.getAnklePositionInFootFrame()
        #self.ankleLength = 0.1935
        #self.ankleWidth  = 0.121

        #self.dynamic.setFootParameters(True , self.ankleLength, self.ankleWidth, ankle)
        #self.dynamic.setFootParameters(False, self.ankleLength, self.ankleWidth, ankle)

        # check half sitting size
        self.dimension = self.dynamic.getDimension()
        if self.dimension != len(self.halfSitting):
            
            raise RuntimeError("invalid half-sitting pose. HalfSitting Dimension:", 
                               len(self.halfSitting), " .Robot Dimension:", self.dimension)
        self.initializeRobot()

__all__ = ["Robot"]
