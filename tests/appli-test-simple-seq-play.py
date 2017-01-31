from dynamic_graph.sot.core.meta_task_posture import MetaTaskKinePosture
from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph.sot.tools import SimpleSeqPlay
from numpy import eye

# Create the posture task
taskKP    = MetaTaskKinePosture(robot.dynamic,'taskPosture')

# Create the sequence player
aSimpleSeqPlay = SimpleSeqPlay('aSimpleSeqPlay')
aSimpleSeqPlay.load("/home/ostasse/devel-src/Talos/src/pyrene-motions/grabHandrail15/stairs_15cm_handrail_grab_actuated")

# Connect the sequence player to the posture task 
from dynamic_graph import plug
plug(aSimpleSeqPlay.posture,taskKP.featureDes.errorIN)

# Create the solver
from dynamic_graph.sot.core import SOT
sot = SOT('sot')
sot.setSize(robot.dynamic.getDimension())
plug(sot.control,robot.device.control)

# Push the posture task in the solver
sot.push(taskKP.task.name)
robot.device.control.recompute(0)
