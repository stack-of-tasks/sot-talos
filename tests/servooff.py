# flake8: noqa
from dynamic_graph import plug
from dynamic_graph.ros import RosPublish

robot.ros = RosPublish('rosPublish')
robot.ros.add('vector', 'robotState_ros', 'robotState')
plug(robot.device.robotState, robot.ros.signal('robotState_ros'))
robot.device.after.addDownsampledSignal('rosPublish.trigger', 1)
