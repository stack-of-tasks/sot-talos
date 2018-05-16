
from dynamic_graph.ros import RosPublish
ros_publish_state = RosPublish ("ros_publish_state")
ros_publish_state.add ("vector", "state", "/sot_hpp/state")
from dynamic_graph import plug
plug (robot.device.state, ros_publish_state.state)
robot.device.after.addDownsampledSignal ("ros_publish_state.trigger", 100)

v=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.00, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
robot.device.control.value=v
