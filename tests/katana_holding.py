# ______________________________________________________________________________
# ******************************************************************************
#
# The simplest robot task: Just go and reach a point
#
# ______________________________________________________________________________
# ******************************************************************************


#-----------------------------------------------------------------------------
#SET THE PATH TO THE URDF AND MESHES
#Define robotName, urdfpath and initialConfig


urdfPath = "/opt/openrobots/share/talos-data"+"/robots/talos_small.urdf"
urdfDir = ["/opt/openrobots/share/talos-data"+"/../"]


dt = 5e-3
robotName = 'TALOS'
OperationalPointsMap = {'left-wrist'  : 'arm_left_7_joint',
                        'right-wrist' : 'arm_right_7_joint',
                        'left-ankle'  : 'leg_left_5_joint',
                        'right-ankle' : 'leg_right_5_joint',
                        'gaze'        : 'head_2_joint',
                        'waist'       : 'root_joint',
                        'chest'       : 'torso_2_joint'}

initial_katana_config = (0., 0.,1.018213, 0., 0., 0. ,
                         0.000093,  0.000406, -0.673653,  0.859395, -0.448041, -0.001708,
                         0.000093,  0.000406, -0.1     ,  0.859395, -0.448041, -0.001708,
                         -0.4     ,  0.006761,
                         0.25847 ,  0.173046, -0.5 , -1.3 , 0. , -0. , 0.1 ,
                         0.2     , -0.173046, 0.0002  , -0.8 ,  0. ,  0. ,  0.1 ,
                         0. , 0.48)

initialConfig = (0.0, 0.0,  1.018213,  0.00  ,  0.0, 0.0,                         #Free flyer
                 0.000093,  0.000406, -0.449102,  0.859395, -0.448041, -0.001708, #Left Leg
                 0.000093,  0.000406, -0.449102,  0.859395, -0.448041, -0.001708, #Right Leg
                 0.0 ,  0.006761,                                                 #Chest
                 0.25847 ,  0.173046, -0.0002, -0.525366, 0.0, -0.0,  0.1,        #Left Arm
                 0.,0.,0.,0.,0.,0.,0.,                                            #Left gripper
                 -0.25847 , -0.173046, 0.0002  , -0.525366, 0.0,  0.0,  0.1,      #Right Arm
                 0.,0.,0.,0.,0.,0.,0.,                                            #Right gripper
                 0.,  0.                                                          #Head
             )

#-----------------------------------------------------------------------------
#---- ROBOT SPECIFICATIONS----------------------------------------------------
#-----------------------------------------------------------------------------


#-----------------------------------------------------------------------------
#---- DYN --------------------------------------------------------------------
#-----------------------------------------------------------------------------
from pinocchio.robot_wrapper import RobotWrapper
import pinocchio as se3
from dynamic_graph.sot.dynamics_pinocchio import fromSotToPinocchio
pinocchioRobot = RobotWrapper(urdfPath, urdfDir, se3.JointModelFreeFlyer())
pinocchioRobot.initDisplay(loadModel=True)
pinocchioRobot.display(fromSotToPinocchio(initial_katana_config))


from dynamic_graph.sot.dynamics_pinocchio.humanoid_robot import HumanoidRobot
robot = HumanoidRobot(robotName, pinocchioRobot.model,
                      pinocchioRobot.data, initial_katana_config, OperationalPointsMap)


# ------------------------------------------------------------------------------
# ---- Kinematic Stack of Tasks (SoT)  -----------------------------------------
# ------------------------------------------------------------------------------
from dynamic_graph import plug
from dynamic_graph.sot.core import SOT
sot = SOT('sot')
sot.setSize(robot.dynamic.getDimension())
plug(sot.control,robot.device.control)

# ------------------------------------------------------------------------------
# ---- TASKS -------------------------------------------------------------------

# ---- TASK GRIP
from dynamic_graph.sot.core.meta_tasks_kine import MetaTaskKine6d, MetaTaskKineCom, gotoNd
from dynamic_graph.sot.core.meta_task_posture import MetaTaskKinePosture
from dynamic_graph.sot.core.matrix_util import matrixToTuple
from numpy import eye

# --- CONTACTS
#define contactLF and contactRF

for name,joint in [ ['LF',robot.OperationalPointsMap['left-ankle']], ['RF',robot.OperationalPointsMap['right-ankle'] ] ]:
    contact = MetaTaskKine6d('contact'+name,robot.dynamic,name,joint)
    contact.feature.frame('desired')
    contact.gain.setConstant(10)
    locals()['contact'+name] = contact

q = fromSotToPinocchio(initial_katana_config)
Mlf_t = pinocchioRobot.position(q,
                              pinocchioRobot.index(OperationalPointsMap["left-ankle"])).translation
Mrf_t = pinocchioRobot.position(q,
                              pinocchioRobot.index(OperationalPointsMap["right-ankle"])).translation
temp = (Mlf_t[2]+Mrf_t[2])/2
Mlf_t[2] = temp
Mrf_t[2]=temp
targetRF = eye(4)
targetLF = eye(4)
targetRF[0:3,3] = Mrf_t.T
targetLF[0:3,3] = Mlf_t.T
gotoNd(contactLF,targetLF,'111111',(4.9,0.9,0.01,0.9))
gotoNd(contactRF,targetRF,'111111',(4.9,0.9,0.01,0.9))


import numpy as np
taskCom = MetaTaskKineCom(robot.dynamic)
robot.dynamic.com.recompute(0)
robot.dynamic.com.recompute(0)
robot.dynamic.leg_left_5_joint.recompute(0)
robot.dynamic.leg_right_5_joint.recompute(0)
target = tuple((np.transpose(robot.dynamic.leg_right_5_joint.value)[3]
                +np.transpose(robot.dynamic.leg_left_5_joint.value)[3])[0:3])
taskCom.featureDes.errorIN.value = target
taskCom.feature.selec.value = '11'
taskCom.task.controlGain.value = 10

taskPosture = MetaTaskKinePosture(robot.dynamic)
robot.dynamic.position.recompute(0)
taskPosture.featureDes.errorIN.value = initial_katana_config
taskPosture.task.controlGain.value = 10

sot.push(contactRF.task.name)
sot.push(contactLF.task.name)
sot.push(taskCom.task.name)
sot.push(taskPosture.task.name)

#-------------------------------------------------------------------------------
#----- MAIN LOOP ---------------------------------------------------------------
#-------------------------------------------------------------------------------

def runner(n):
    for i in xrange(n):
        robot.device.increment(dt)
        pinocchioRobot.display(fromSotToPinocchio(robot.device.state.value))

runner(1000)

np.set_printoptions(suppress=True)

#final_katana_config = tuple(np.round(np.matrix(robot.dynamic.position.value), decimals=6)[0])

final_katana_config = (-0.055354,  0.000556,  1.016732, -0.001192, -0.101143, -0.001338,
                       0.001326,  0.001261, -0.599846,  0.825254, -0.124272, -0.001435,
                       0.001324,  0.001256, -0.065729,  0.914321, -0.747456, -0.001435,
                       -0.401101, -0.045825,
                       0.270664,  0.175307, -0.502392, -1.298519, 0.000007, -0.000075,  0.100024,
                       0.185163, -0.172731,  0.000315, -0.795656,  0.000036,  0.000002,  0.10012,
                       -0.002262,  0.479947)
