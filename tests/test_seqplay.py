# ______________________________________________________________________________
# ******************************************************************************
#
# The simplest robot task: Just go and reach a point
#
# ______________________________________________________________________________
# ******************************************************************************

import pinocchio as se3
from numpy import hstack, identity, zeros
from pinocchio.robot_wrapper import RobotWrapper

from dynamic_graph import plug
from dynamic_graph.sot.core.feature_generic import FeatureGeneric
from dynamic_graph.sot.core.gain_adaptive import GainAdaptive
from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph.sot.core.meta_tasks import setGain
from dynamic_graph.sot.core.operator import Selec_of_vector
from dynamic_graph.sot.core.sot import SOT, Task
from dynamic_graph.sot.dynamic_pinocchio import fromSotToPinocchio
from dynamic_graph.sot.dynamic_pinocchio.humanoid_robot import HumanoidRobot
from dynamic_graph.sot.tools import SimpleSeqPlay

# -----------------------------------------------------------------------------
# SET THE PATH TO THE URDF AND MESHES
# Define robotName, urdfpath and initialConfig

# Make sure talos_description is in the ROS_PACKAGE_PATH
# from rospkg import RosPack
# rospack = RosPack()
# urdfPath = rospack.get_path('talos_description')+"/robots/talos_full_collision.urdf"
# urdfDir = [rospack.get_path('talos_description')+"/../"]

URDFPATH = "~/git/pyrene/talos-data" + "/robots/talos_reduced.urdf"
URDFDIR = ["~/git/pyrene/talos-data" + "/../"]
MOTION_SEQUENCE = "~/git/pyrene/pyrene-motions/grabHandrail15/stairs_15cm_handrail_grab_actuated"
DISPLAY = True

dt = 1e-3
robotName = 'TALOS'
OperationalPointsMap = {
    'left-wrist': 'arm_left_7_joint',
    'right-wrist': 'arm_right_7_joint',
    'left-ankle': 'leg_left_6_joint',
    'right-ankle': 'leg_right_6_joint',
    'gaze': 'head_2_joint',
    'waist': 'root_joint',
    'chest': 'torso_2_joint'
}

halfSitting = (
    0.0,
    0.0,
    1.018213,
    0.00,
    0.0,
    0.0,  # Free flyer
    0.0,
    0.0,
    -0.411354,
    0.859395,
    -0.448041,
    -0.001708,  # Left Leg
    0.0,
    0.0,
    -0.411354,
    0.859395,
    -0.448041,
    -0.001708,  # Right Leg
    0.0,
    0.006761,  # Chest
    0.25847,
    0.173046,
    -0.0002,
    -0.525366,
    0.0,
    -0.0,
    0.1,
    0.1,  # Left Arm
    -0.25847,
    -0.173046,
    0.0002,
    -0.525366,
    0.0,
    0.0,
    0.1,
    0.1,  # Right Arm
    0.,
    0.  # Head
)

# -----------------------------------------------------------------------------
# ---- ROBOT SPECIFICATIONS----------------------------------------------------
# -----------------------------------------------------------------------------

pinocchioRobot = RobotWrapper(URDFPATH, URDFDIR, se3.JointModelFreeFlyer())

pinocchioRobot.initDisplay(loadModel=DISPLAY)
if DISPLAY:
    pinocchioRobot.display(fromSotToPinocchio(halfSitting))

robot = HumanoidRobot(robotName, pinocchioRobot.model, pinocchioRobot.data, halfSitting, OperationalPointsMap)

sot = SOT('sot')
sot.setSize(robot.dynamic.getDimension())
plug(sot.control, robot.device.control)

task_name = "posture_task"
taskPosture = Task(task_name)
taskPosture.dyn = robot.dynamic
taskPosture.feature = FeatureGeneric('feature_' + task_name)
taskPosture.featureDes = FeatureGeneric('feature_des_' + task_name)
taskPosture.gain = GainAdaptive("gain_" + task_name)
robotDim = robot.dynamic.getDimension()
first_6 = zeros((32, 6))
other_dof = identity(robotDim - 6)
jacobian_posture = hstack([first_6, other_dof])
taskPosture.feature.jacobianIN.value = matrixToTuple(jacobian_posture)
taskPosture.feature.setReference(taskPosture.featureDes.name)
taskPosture.add(taskPosture.feature.name)

seqplay = SimpleSeqPlay("seq_play")
seqplay.load(MOTION_SEQUENCE)

plug(seqplay.posture, taskPosture.featureDes.errorIN)
getPostureValue = Selec_of_vector("current_posture")
getPostureValue.selec(6, robotDim)
plug(robot.dynamic.position, getPostureValue.sin)
plug(getPostureValue.sout, taskPosture.feature.errorIN)
plug(getPostureValue.sout, seqplay.currentPosture)
setGain(taskPosture.gain, (4.9, 0.9, 0.01, 0.9))
plug(taskPosture.gain.gain, taskPosture.controlGain)
plug(taskPosture.error, taskPosture.gain.error)

# START SEQUENCE PLAYER
seqplay.start()
taskPosture.featureDes.errorIN.recompute(0)

# PUSH TO SOLVER
sot.push(taskPosture.name)

# -------------------------------------------------------------------------------
# ----- MAIN LOOP ---------------------------------------------------------------
# -------------------------------------------------------------------------------


def runner(n):
    for i in range(n):
        robot.device.increment(dt)
        pinocchioRobot.display(fromSotToPinocchio(robot.device.state.value))


runner(3000)
