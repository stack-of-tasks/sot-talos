# -*- coding: utf-8 -*-
# Copyright 2016, Olivier Stasse, CNRS

from dynamic_graph.sot.talos import Talos


class Robot(Talos):
    """
    This class instantiates LAAS TALOS Robot
    """

    def defineHalfSitting(self, q):
        """
        q is the configuration to fill.

        When this function is called,
        the attribute pinocchioModel has been filled.
        """
        # Free flyer
        q[2] = 1.018213
        # left leg
        self.setJointValueInConfig(q,
                                   ["leg_left_{}_joint".format(i+1)
                                    for i in range(6)],
                                   [0., 0., -0.411354, 0.859395,
                                    -0.448041, -0.001708])
        # right leg
        self.setJointValueInConfig(q,
                                   ["leg_right_{}_joint".format(i+1)
                                    for i in range(6)],
                                   [0., 0., -0.411354, 0.859395,
                                    -0.448041, -0.001708])
        # torso
        self.setJointValueInConfig(q,
                                   ["torso_{}_joint".format(i+1)
                                    for i in range(2)],
                                   [0., 0.006761])
        # left arm
        self.setJointValueInConfig(q,
                                   ["arm_left_{}_joint".format(i+1)
                                    for i in range(7)],
                                   [0.25847, 0.173046, -0.0002,
                                    -0.525366, 0., -0., 0.1])
        # right arm
        self.setJointValueInConfig(q,
                                   ["arm_right_{}_joint".format(i+1)
                                    for i in range(7)],
                                   [-0.25847, -0.173046, 0.0002,
                                    -0.525366, 0., 0., 0.1])
        # grippers
        self.setJointValueInConfig(q,
                                   ["gripper_left_joint",
                                    "gripper_right_joint"],
                                   [-0.005, -0.005])
        # torso
        self.setJointValueInConfig(q,
                                   ["head_{}_joint".format(i+1)
                                    for i in range(2)],
                                   [0., 0.])

    def __init__(self, name, device=None, tracer=None, fromRosParam=None):
        Talos.__init__(self, name, device=device, tracer=tracer,
                       fromRosParam=fromRosParam)
        """
        TODO:Confirm these values
        # Define camera positions w.r.t gaze.
        cameraBottomLeftPosition = np.matrix((
            (0.98481, 0.00000, 0.17365, 0.035),
            (0.,      1.,      0.,      0.072),
            (-0.17365, 0.00000, 0.98481, 0.075 - 0.03),
            (0., 0., 0., 1.),
        ))
        cameraBottomRightPosition = np.matrix((
            (0.98481, 0.00000, 0.17365, 0.035),
                (0.,      1.,      0.,     -0.072),
                (-0.17365, 0.00000, 0.98481, 0.075 - 0.03),
                (0., 0., 0., 1.),
                ))
        cameraTopLeftPosition = np.matrix((
            (0.99920,  0.00120, 0.03997, 0.01),
            (0.00000,  0.99955,-0.03000, 0.029),
            (-0.03999, 0.02997, 0.99875, 0.145 - 0.03),
            (0.,       0.,      0.,      1.),
        ))
        cameraTopRightPosition = np.matrix((
            (0.99920,  0.00000, 0.03999,  0.01),
            (0.00000,  1.00000, 0.00000, -0.029),
            (-0.03999, 0.00000, 0.99920,  0.145 - 0.03),
            (0.,       0.,      0.,       1.),
        ))
        # Frames re-orientation:
        # Z = depth (increase from near to far)
        # X = increase from left to right
        # Y = increase from top to bottom

        c1_M_c = np.matrix(
            [[ 0.,  0.,  1., 0.],
             [-1.,  0.,  0., 0.],
             [ 0., -1.,  0., 0.],
             [ 0.,  0.,  0., 1.]])

        for camera in [cameraBottomLeftPosition, cameraBottomRightPosition,
                       cameraTopLeftPosition, cameraTopRightPosition]:
            camera[:] = camera * c1_M_c

        self.AdditionalFrames.append(
            ("cameraBottomLeft",
             matrixToTuple(cameraBottomLeftPosition), "gaze"))
        self.AdditionalFrames.append(
            ("cameraBottomRight",
             matrixToTuple(cameraBottomRightPosition), "gaze"))
        self.AdditionalFrames.append(
            ("cameraTopLeft",
             matrixToTuple(cameraTopLeftPosition), "gaze"))
        self.AdditionalFrames.append(
            ("cameraTopRight",
             matrixToTuple(cameraTopRightPosition), "gaze"))
        """


__all__ = ["Robot"]
