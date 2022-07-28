# flake8: noqa
# from dynamic_graph.sot.core.meta_task_posture import MetaTaskKinePosture
from numpy import eye, hstack, identity, zeros

from dynamic_graph import plug
from dynamic_graph.sot.core.feature_generic import FeatureGeneric
from dynamic_graph.sot.core.gain_adaptive import GainAdaptive
from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph.sot.core.meta_tasks import setGain
from dynamic_graph.sot.core.operator import Selec_of_vector

# Create the solver
# Connects the sequence player to the posture task
from dynamic_graph.sot.core.sot import SOT, Task
from dynamic_graph.sot.tools import SimpleSeqPlay

# Create the posture task
task_name = "posture_task"
taskPosture = Task(task_name)
taskPosture.dyn = robot.dynamic
taskPosture.feature = FeatureGeneric("feature_" + task_name)
taskPosture.featureDes = FeatureGeneric("feature_des_" + task_name)
taskPosture.gain = GainAdaptive("gain_" + task_name)
robotDim = robot.dynamic.getDimension()
first_6 = zeros((32, 6))
other_dof = identity(robotDim - 6)
jacobian_posture = hstack([first_6, other_dof])
taskPosture.feature.jacobianIN.value = matrixToTuple(jacobian_posture)
taskPosture.feature.setReference(taskPosture.featureDes.name)
taskPosture.add(taskPosture.feature.name)

# Create the sequence player
aSimpleSeqPlay = SimpleSeqPlay("aSimpleSeqPlay")
aSimpleSeqPlay.load(
    "/home/ostasse/devel-src/robotpkg-test3/install/share/pyrene-motions/identification/OEM_arms_60s_500Hz"
)
aSimpleSeqPlay.setTimeToStart(10.0)

plug(aSimpleSeqPlay.posture, taskPosture.featureDes.errorIN)

# Connects the dynamics to the current feature of the posture task
getPostureValue = Selec_of_vector("current_posture")
getPostureValue.selec(6, robotDim)
plug(robot.dynamic.position, getPostureValue.sin)
plug(getPostureValue.sout, taskPosture.feature.errorIN)
plug(getPostureValue.sout, aSimpleSeqPlay.currentPosture)

# Set the gain of the posture task
setGain(taskPosture.gain, (4.9, 0.9, 0.01, 0.9))
plug(taskPosture.gain.gain, taskPosture.controlGain)
plug(taskPosture.error, taskPosture.gain.error)

sot = SOT("sot")
sot.setSize(robot.dynamic.getDimension())
plug(sot.control, robot.device.control)

taskPosture.featureDes.errorIN.recompute(0)

# Push the posture task in the solver
