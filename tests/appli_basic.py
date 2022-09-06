# flake8: noqa
from dynamic_graph import plug
from dynamic_graph.ros import RosPublish

ros_publish_state = RosPublish("ros_publish_state")
ros_publish_state.add("vector", "state", "/sot_hpp/state")
plug(robot.device.state, ros_publish_state.state)
robot.device.after.addDownsampledSignal("ros_publish_state.trigger", 100)

v = (
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.00,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
)
robot.device.control.value = v
