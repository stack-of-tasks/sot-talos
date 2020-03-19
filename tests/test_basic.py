#!/usr/bin/python
# flake8: noqa
import rospy
from dynamic_graph_bridge_msgs.srv import *
from std_srvs.srv import *

try:
    # Python 2
    input = raw_input
except NameError:
    pass


def launchScript(code, title, description=""):
    input(title + ':   ' + description)
    rospy.loginfo(title)
    rospy.loginfo(code)
    for line in code:
        if line != '' and line[0] != '#':
            print(line)
            answer = runCommandClient(str(line))
            rospy.logdebug(answer)
            print(answer)
    rospy.loginfo("...done with " + title)


# Waiting for services
try:
    rospy.loginfo("Waiting for run_command")
    rospy.wait_for_service('/run_command')
    rospy.loginfo("...ok")

    rospy.loginfo("Waiting for start_dynamic_graph")
    rospy.wait_for_service('/start_dynamic_graph')
    rospy.loginfo("...ok")

    runCommandClient = rospy.ServiceProxy('run_command', RunCommand)
    runCommandStartDynamicGraph = rospy.ServiceProxy('start_dynamic_graph', Empty)

    initCode = open("appli_basic.py", "r").read().split("\n")

    rospy.loginfo("Stack of Tasks launched")

    # runCommandClient("from dynamic_graph import plug")
    # runCommandClient("from dynamic_graph.sot.core.sot import SOT")
    # runCommandClient("sot = SOT('sot')")
    # runCommandClient("sot.setSize(robot.dynamic.getDimension())")
    # runCommandClient("plug(sot.control,robot.device.control)")

    launchScript(initCode, 'initialize SoT')
    input("Wait before starting the dynamic graph")
    runCommandStartDynamicGraph()

except rospy.ServiceException as e:
    rospy.logerr("Service call failed: %s" % e)
