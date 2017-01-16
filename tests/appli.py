from dynamic_graph.sot.core.meta_tasks_kine import MetaTaskKine6d, MetaTaskKineCom, gotoNd
from dynamic_graph.sot.core.matrix_util import matrixToTuple
from numpy import eye

taskRH    = MetaTaskKine6d('rh',robot.dynamic,'rh',robot.OperationalPointsMap['right-wrist'])
handMgrip = eye(4); handMgrip[0:3,3] = (0.1,0,0)
taskRH.opmodif = matrixToTuple(handMgrip)
taskRH.feature.frame('desired')
# --- STATIC COM (if not walking)
taskCom = MetaTaskKineCom(robot.dynamic)
robot.dynamic.com.recompute(0)
taskCom.featureDes.errorIN.value = robot.dynamic.com.value
taskCom.task.controlGain.value = 10


# --- CONTACTS
#define contactLF and contactRF
for name,joint in [ ['LF',robot.OperationalPointsMap['left-ankle']], ['RF',robot.OperationalPointsMap['right-ankle'] ] ]:
    contact = MetaTaskKine6d('contact'+name,robot.dynamic,name,joint)
    contact.feature.frame('desired')
    contact.gain.setConstant(10)
    contact.keep()
    locals()['contact'+name] = contact


from dynamic_graph import plug
from dynamic_graph.sot.core import SOT
sot = SOT('sot')
sot.setSize(robot.dynamic.getDimension())
plug(sot.control,robot.device.control)



target = (0.5,-0.2,1.0)

#addRobotViewer(robot, small=False)
#robot.viewer.updateElementConfig('zmp',target+(0,0,0))



gotoNd(taskRH,target,'111',(4.9,0.9,0.01,0.9))
sot.push(contactRF.task.name)
sot.push(contactLF.task.name)
sot.push(taskCom.task.name)
sot.push(taskRH.task.name)
robot.device.control.recompute(0)
